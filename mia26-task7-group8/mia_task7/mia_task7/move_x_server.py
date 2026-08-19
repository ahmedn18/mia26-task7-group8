#!/usr/bin/env python3
"""move_x action server.

Drives the robot a signed distance along its current heading, positive
forward and negative in reverse. Publishes to /cmd_vel, tracks position on
/odom, and succeeds only once the requested distance has been covered.

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

from turtlebot_interface.action import MoveX

from mia_task7.maze_config import (
    CONTROL_HZ,
    GOAL_TIMEOUT_S,
    LINEAR_KP,
    LINEAR_TOLERANCE_M,
    MAX_LINEAR_SPEED,
    MIN_LINEAR_SPEED,
    ODOM_STARTUP_TIMEOUT_S,
    ODOM_TIMEOUT_S,
)


class MoveXActionServer(Node):
    """Serves the move_x action."""

    def __init__(self):
        super().__init__('move_x_server')

        # Reentrant so /odom keeps being delivered while a goal is executing.
        self.callback_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            MoveX,
            'move_x',
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
        self.position = None
        self.last_odom_time = None

        self.get_logger().info('move_x action server ready.')

    def odom_callback(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        with self._lock:
            self.position = (position.x, position.y)
            self.last_odom_time = self.get_clock().now()

    def _read_odom(self):
        with self._lock:
            return self.position, self.last_odom_time

    def _wait_for_first_odom(self) -> bool:
        """Block briefly for the first /odom message. False if none arrives."""
        deadline = time.monotonic() + ODOM_STARTUP_TIMEOUT_S
        while time.monotonic() < deadline:
            position, _ = self._read_odom()
            if position is not None:
                return True
            time.sleep(1.0 / CONTROL_HZ)
        return False

    def stop_robot(self) -> None:
        self.cmd_vel_pub.publish(Twist())

    def _abort(self, goal_handle, message: str, travelled: float):
        self.get_logger().error(message)
        self.stop_robot()
        goal_handle.abort()
        return MoveX.Result(
            success=False, message=message, distance_travelled=travelled)

    def execute_callback(self, goal_handle):
        target_distance = goal_handle.request.target_distance
        self.get_logger().info(f'move_x goal: {target_distance:.3f} m')

        if not self._wait_for_first_odom():
            return self._abort(
                goal_handle,
                'No /odom received -- cannot measure distance.',
                0.0)

        start_position, _ = self._read_odom()
        start_x, start_y = start_position

        # Displacement is a magnitude, so the sign of the goal decides the
        # direction we drive and we compare against its absolute value.
        goal_magnitude = abs(target_distance)
        direction = math.copysign(1.0, target_distance)

        travelled = 0.0
        start_time = self.get_clock().now()
        feedback = MoveX.Feedback()
        cmd = Twist()

        # time.sleep is safe here only because main() runs a
        # MultiThreadedExecutor and the subscription shares a reentrant
        # callback group -- /odom is delivered on another thread while this
        # one sleeps.
        period = 1.0 / CONTROL_HZ

        while rclpy.ok():
            if not goal_handle.is_active:
                self.stop_robot()
                return MoveX.Result(
                    success=False, message='Goal no longer active.',
                    distance_travelled=travelled)

            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > GOAL_TIMEOUT_S:
                return self._abort(
                    goal_handle,
                    f'Timed out after {elapsed:.1f}s with '
                    f'{goal_magnitude - travelled:.3f} m still to cover.',
                    travelled)

            position, last_odom_time = self._read_odom()
            odom_age = (
                self.get_clock().now() - last_odom_time).nanoseconds / 1e9
            if odom_age > ODOM_TIMEOUT_S:
                return self._abort(
                    goal_handle,
                    f'/odom went silent {odom_age:.1f}s ago.',
                    travelled)

            travelled = math.dist(position, (start_x, start_y))

            remaining = goal_magnitude - travelled
            if remaining <= LINEAR_TOLERANCE_M:
                break

            # Proportional, so the robot eases into the target instead of
            # overshooting it at full speed.
            speed = min(MAX_LINEAR_SPEED,
                        max(MIN_LINEAR_SPEED, LINEAR_KP * remaining))
            cmd.linear.x = direction * speed
            self.cmd_vel_pub.publish(cmd)

            feedback.current_distance = travelled
            goal_handle.publish_feedback(feedback)

            time.sleep(period)

        self.stop_robot()
        goal_handle.succeed()

        self.get_logger().info(f'move_x done: covered {travelled:.3f} m.')

        return MoveX.Result(
            success=True,
            message=f'Covered {travelled:.3f} m.',
            distance_travelled=math.copysign(travelled, direction))


def main(args=None):
    rclpy.init(args=args)
    node = MoveXActionServer()

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
