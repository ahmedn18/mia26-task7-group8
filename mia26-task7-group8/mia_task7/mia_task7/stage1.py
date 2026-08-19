import asyncio
import math

from rclpy.node import Node
from rclpy.action import ActionClient
from std_srvs.srv import SetBool

from maze_nav_interfaces.action import MoveX, MoveYaw


GATE_OPEN_S = 2.0


class Stage1:

    def __init__(self, node: Node, gate_client):

        self.node = node
        self.gate_client = gate_client

        # Action servers
        self.move_yaw_client = ActionClient(
            node,
            MoveYaw,
            'move_yaw'
        )

        self.move_x_client = ActionClient(
            node,
            MoveX,
            'move_x'
        )

        async def run(self) -> bool:


if not await self._call_gate(True):
    self.node.get_logger().error(
        "Stage 1: gate service call failed"
    )
    return False

