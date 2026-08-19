#!/usr/bin/env python3
"""move_yaw action server.

Rotates the robot by a signed angle in radians, positive turning left (CCW).
Publishes to /cmd_vel, tracks orientation on /odom, and succeeds only once
the requested angle has been turned.

Progress is measured by accumulating the per-tick change in yaw rather than
by comparing the current yaw against an absolute target. Odometry yaw comes
out of atan2 and so jumps between +pi and -pi; accumulating wrapped deltas
means that discontinuity costs nothing and turns larger than half a circle
still terminate.

Edge cases handled, per the task brief:
  * /odom never arrives, or stops arriving mid-goal -> abort
  * the goal does not complete in time                -> abort
Both stop the robot before aborting.
"""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from turtlebot_interface.action import MoveYaw

from mia_task7.maze_config import (
    CONTROL_HZ,
    GOAL_TIMEOUT_S,
    MAX_YAW_SPEED,
    MIN_YAW_SPEED,
    ODOM_STARTUP_TIMEOUT_S,
    ODOM_TIMEOUT_S,
    YAW_KP,
    YAW_TOLERANCE_RAD,
)


def normalize_angle(angle: float) -> float:
    """Wrap an angle into (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class MoveYawActionServer(Node):
    """Serves the move_yaw action."""

    def __init__(self):
        super().__init__('move_yaw_server')

        # Reentrant so /odom keeps being delivered while a goal is executing.
        # With the default group the callbacks serialise and the goal never
        # sees the odometry it is waiting on.
        self.callback_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            MoveYaw,
            'move_yaw',
            self.execute_callback,
            callback_group=self.callback_group)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
            callback_group=self.callback_group)

        # Written from the odom callback, read from the goal thread.
        self._lock = threading.Lock()
        self.current_yaw = None
        self.last_odom_time = None

        self.get_logger().info('move_yaw action server ready.')

    def odom_callback(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        with self._lock:
            self.current_yaw = yaw
            self.last_odom_time = self.get_clock().now()

    def _read_odom(self):
        with self._lock:
            return self.current_yaw, self.last_odom_time

    def _wait_for_first_odom(self) -> bool:
        """Block briefly for the first /odom message. False if none arrives."""
        deadline = time.monotonic() + ODOM_STARTUP_TIMEOUT_S
        while time.monotonic() < deadline:
            yaw, _ = self._read_odom()
            if yaw is not None:
                return True
            time.sleep(1.0 / CONTROL_HZ)
        return False

    def stop_robot(self) -> None:
        self.cmd_vel_pub.publish(Twist())

    def _abort(self, goal_handle, message: str, angle_turned: float):
        self.get_logger().error(message)
        self.stop_robot()
        goal_handle.abort()
        return MoveYaw.Result(
            success=False, message=message, angle_turned=angle_turned)

    def execute_callback(self, goal_handle):
        target_yaw = goal_handle.request.target_yaw
        self.get_logger().info(
            f'move_yaw goal: {target_yaw:.3f} rad '
            f'({math.degrees(target_yaw):.1f} deg)')

        if not self._wait_for_first_odom():
            return self._abort(
                goal_handle,
                'No /odom received -- cannot measure rotation.',
                0.0)

        last_yaw, _ = self._read_odom()
        turned = 0.0
        start_time = self.get_clock().now()
        feedback = MoveYaw.Feedback()
        cmd = Twist()

        # time.sleep is safe here only because main() runs a
        # MultiThreadedExecutor and the subscription shares a reentrant
        # callback group -- /odom is delivered on another thread while this
        # one sleeps.
        period = 1.0 / CONTROL_HZ

        while rclpy.ok():
            if not goal_handle.is_active:
                self.stop_robot()
                return MoveYaw.Result(
                    success=False, message='Goal no longer active.',
                    angle_turned=turned)

            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > GOAL_TIMEOUT_S:
                return self._abort(
                    goal_handle,
                    f'Timed out after {elapsed:.1f}s with '
                    f'{target_yaw - turned:.3f} rad still to turn.',
                    turned)

            yaw_now, last_odom_time = self._read_odom()
            odom_age = (
                self.get_clock().now() - last_odom_time).nanoseconds / 1e9
            if odom_age > ODOM_TIMEOUT_S:
                return self._abort(
                    goal_handle,
                    f'/odom went silent {odom_age:.1f}s ago.',
                    turned)

            # Accumulate the wrapped delta, so crossing +pi/-pi is a non-event.
            turned += normalize_angle(yaw_now - last_yaw)
            last_yaw = yaw_now

            remaining = target_yaw - turned
            if abs(remaining) <= YAW_TOLERANCE_RAD:
                break

            # Proportional, so the robot eases into the target instead of
            # overshooting it at full speed.
            speed = min(MAX_YAW_SPEED, max(MIN_YAW_SPEED, YAW_KP * abs(remaining)))
            cmd.angular.z = math.copysign(speed, remaining)
            self.cmd_vel_pub.publish(cmd)

            feedback.current_yaw = turned
            goal_handle.publish_feedback(feedback)

            time.sleep(period)

        self.stop_robot()
        goal_handle.succeed()

        self.get_logger().info(
            f'move_yaw done: turned {turned:.3f} rad '
            f'({math.degrees(turned):.1f} deg).')

        return MoveYaw.Result(
            success=True,
            message=f'Turned {turned:.3f} rad.',
            angle_turned=turned)


def main(args=None):
    rclpy.init(args=args)
    node = MoveYawActionServer()

    # Required: the goal thread sleeps, so callbacks need other threads.
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
