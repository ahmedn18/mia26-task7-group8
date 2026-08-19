 
import asyncio
import math
 
from rclpy.node import Node
from rclpy.action import ActionClient
from std_srvs.srv import SetBool
 
# Placeholder import path - update to match your team's actual
# interface package name once it exists.
from maze_nav_interfaces.action import MoveX, MoveYaw
 
GATE_OPEN_S = 2.0  # budget for the wall to finish moving (README)
 
 
class Stage1:
    """
    Owns: request gate 1 -> yaw +90 -> +y 0.30 -> hold -> +y 0.45
    through gate 1. Ends at y ~= 1.25.
    """
 
    def __init__(self, node: Node, move_x_client: ActionClient,
                 move_yaw_client: ActionClient, gate_client):
        self.node = node
        self.move_x_client = move_x_client
        self.move_yaw_client = move_yaw_client
        self.gate_client = gate_client
 
    async def run(self) -> bool:
        """Runs the whole stage. Returns True on success, False on failure."""

if not await self._call_gate(open_gate1=True):
            self.node.get_logger().error("Stage 1: gate service call failed")
            return False


if not await self._send_yaw(math.radians(90)):
            self.node.get_logger().error("Stage 1: yaw +90 failed")
            return False

if not await self._send_x(0.30):
            self.node.get_logger().error("Stage 1: approach leg failed")
            return False
