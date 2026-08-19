"""Stage 3: turn east and run to the finish pad.

Stage 2 leaves the robot at roughly (0.5, 2.5) still facing +y, above gate 2
and inside the band (y >= 2) where the east side of the arena is open. This
stage turns right onto that opening and drives the length of the arena to the
pad centred on (5.0, 2.475).

Both gates are behind the robot by now, so this stage never touches the wall
service. It still takes gate_client so that every stage has the same
constructor and solve_maze can load them uniformly.

The run is split into a long approach and a short touch. The simulation stops
itself the instant the robot reaches the pad, which cuts off whatever goal is
in flight -- /odom stops and the action server aborts on a silent sensor. By
keeping that exposure to the final 0.30 m, the 4.20 m approach is verified
normally and only the touch is allowed to end that way.
"""

from rclpy.action import ActionClient
from rclpy.node import Node

from turtlebot_interface.action import MoveX, MoveYaw

from mia_task7.maze_config import (
    STAGE3_APPROACH_M,
    STAGE3_TOUCH_M,
    STAGE3_TURN_RAD,
)

SERVICE_TIMEOUT_S = 2.0


class Stage3:
    """Final leg of the run."""

    def __init__(self, node: Node, gate_client):
        self.node = node
        # Unused: both gates are behind us. Kept for a uniform stage signature.
        self.gate_client = gate_client

        self.move_x_client = ActionClient(node, MoveX, 'move_x')
        self.move_yaw_client = ActionClient(node, MoveYaw, 'move_yaw')

    async def run(self) -> bool:
        if not await self._send_yaw(STAGE3_TURN_RAD):
            self.node.get_logger().error('Stage 3: yaw -90 failed')
            return False

        if not await self._send_x(STAGE3_APPROACH_M):
            self.node.get_logger().error('Stage 3: run to the pad failed')
            return False

        self.node.get_logger().info(
            'Stage 3: at the pad edge, touching the finish')

        # From here the simulation may shut down under us at any moment, and
        # that shutdown is the win condition rather than a fault. Report it
        # as such instead of failing the run.
        if not await self._send_x(STAGE3_TOUCH_M):
            self.node.get_logger().warn(
                'Stage 3: the final goal did not report success. That is '
                'expected if the pad was reached -- the simulation stops on '
                'contact, which cuts the goal off. Check the Gazebo window.')
            return True

        self.node.get_logger().info('Stage 3 complete, on the finish pad')
        return True

    async def _send_yaw(self, angle_rad: float) -> bool:
        if not self.move_yaw_client.wait_for_server(
                timeout_sec=SERVICE_TIMEOUT_S):
            self.node.get_logger().error('move_yaw server is not available')
            return False

        goal_msg = MoveYaw.Goal()
        goal_msg.target_yaw = angle_rad

        goal_handle = await self.move_yaw_client.send_goal_async(goal_msg)

        if not goal_handle.accepted:
            self.node.get_logger().error('move_yaw goal was rejected')
            return False

        result = await goal_handle.get_result_async()

        return bool(result.result.success)

    async def _send_x(self, distance_m: float) -> bool:
        if not self.move_x_client.wait_for_server(
                timeout_sec=SERVICE_TIMEOUT_S):
            self.node.get_logger().error('move_x server is not available')
            return False

        goal_msg = MoveX.Goal()
        goal_msg.target_distance = distance_m

        goal_handle = await self.move_x_client.send_goal_async(goal_msg)

        if not goal_handle.accepted:
            self.node.get_logger().error('move_x goal was rejected')
            return False

        result = await goal_handle.get_result_async()

        return bool(result.result.success)
