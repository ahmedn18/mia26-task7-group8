import rclpy
from rclpy import Node
from robot_action.action import Move
from rclpy.action import ActionClient
from std_srvs/srv/SetBool import SetBool
class Action_client(Node):
    def __init__(self):

        #create action client 
        super().__init__('move_client')
        self.action_client = rclpy.action.ActionClient(
            self, 
            Move, 
            '/move_yaw')
        
    def send_goal(self, first_yaw: float, second_yaw: float):
        self.get_logger().info('Waiting for action server...')
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available, shutting down')
            rclpy.shutdown()
            return

        #our goal is  
        goal_msg = Move.Goal()
        goal_msg.first_yaw = first_yaw
        goal_msg.second_yaw = second_yaw

        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
            self.get_logger().info(
            f'Feedback: {feedback.current_action}, progress: {feedback.progress:.0%}'
        )
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result received: Success: {result.success}, Message: {result.message}')
        rclpy.shutdown()
def main():
    rclpy.init()
    node = MoveYawClient()
    node.send_goal(first_yaw=90.0, second_yaw=0.0)
    rclpy.spin(node)
