"""Stage 2: through gate 2, ending lined up with the finish pad.

Stage 1 leaves the robot at roughly (0.5, 1.25) still facing +y, past gate 1.
This stage drives it up through gate 2 at y = 2.0 and stops at y ~= 2.5, in
the band (y >= 2) where the east side of the arena is open.

No rotation here -- the robot is already pointing the right way, and stage 3
does the turn onto the finish run.
"""

import asyncio
import time

from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import SetBool

from turtlebot_interface.action import MoveX

from mia_task7.maze_config import (
    GATE_OPEN_S,
    STAGE2_APPROACH_M,
    STAGE2_CROSS_M,
)

SERVICE_TIMEOUT_S = 2.0


class Stage2:
    """Second leg of the run."""

    def __init__(self, node: Node, gate_client):
        self.node = node
        self.gate_client = gate_client

        self.move_x_client = ActionClient(node, MoveX, 'move_x')

        # Set when the gate request goes out, so the hold below only waits
        # out whatever is left of the wall's travel.
        self._gate_requested_at = None

    async def run(self) -> bool:
        # data: false opens gate 2 and closes gate 1 behind us. That is only
        # safe because stage 1 finished at y ~= 1.25, already clear of gate 1.
        if not await self._request_gate(False):
            self.node.get_logger().error(
                'Stage 2: gate service call failed')
            return False

        if not await self._send_x(STAGE2_APPROACH_M):
            self.node.get_logger().error(
                'Stage 2: approach movement failed')
            return False

        await self._wait_out_gate()

        if not await self._send_x(STAGE2_CROSS_M):
            self.node.get_logger().error(
                'Stage 2: crossing movement failed')
            return False

        self.node.get_logger().info('Stage 2 complete, y ~= 2.5')
        return True

    async def _request_gate(self, open_gate_1: bool) -> bool:
        """Ask for the gate and note when, without waiting for it to move."""
        if not self.gate_client.wait_for_service(
                timeout_sec=SERVICE_TIMEOUT_S):
            self.node.get_logger().error('Gate service is not available')
            return False

        request = SetBool.Request()
        request.data = open_gate_1

        self._gate_requested_at = time.monotonic()
        result = await self.gate_client.call_async(request)

        return bool(result.success)

    async def _wait_out_gate(self) -> None:
        """Hold until the wall has actually finished travelling.

        The service reports success as soon as the command is published, so
        the only thing that says the gate is clear is elapsed time. Waiting
        the remainder rather than the full GATE_OPEN_S means a slow approach
        leg costs nothing extra.
        """
        if self._gate_requested_at is None:
            remaining = GATE_OPEN_S
        else:
            remaining = GATE_OPEN_S - (time.monotonic() - self._gate_requested_at)

        if remaining > 0.0:
            self.node.get_logger().info(
                f'Stage 2: holding {remaining:.2f}s for gate 2')
            await asyncio.sleep(remaining)

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
