import rclpy
from rclpy import Node
from turtleBot_interface.action import move
from geometery_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Action_client(Node):
    def __init__(self):

        #create action client 
        super().__init__('move_client')
        self.action_client = rclpy.action.ActionClient(
            self, 
            move, 
            '/move
            '
        )

        """#publishing to cmd_vel 
        self.publisher_ = self.create_publisher(
            Twist ,
            '/cmd_vel',
            10
        ) 

        #subscribe to odom
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10 
        )
    #callback function to save odom data ( now i have the odom data in the callback function, but i need to save it to use it later)
"""
   """ def odom_callback(self, msg):
        pos = msg.pose.pose.position
        
        ori = msg.pose.pose.orientation
        
        linear_vel = msg.twist.twist.linear

            f"Pos: X={pos.x:.2f}, Y={pos.y:.2f} | "
            f"Ori: W={ori.w:.2f} | "
            f"Speed: X={linear_vel.x:.2f} m/s"
        self.get_logger().info(f"Pos: X={pos.x:.2f}, Y={pos.y:.2f} | Ori: W={ori.w:.2f} | Speed: X={linear_vel.x:.2f} m/s") #publishing the postion in terminal

        )"""
    def send_goal(self):
        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server(timeout_sec=10.0)
        goal_msg = move_yaw.Goal()

        #our goal is  
        goal_msg.firstYaw = 90.0
        goal_msg.secondYaw = 0.0

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
        self.get_logger().info(f'Feedback received: {feedback.current_action}, Progress: {feedback.progress}%')
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result received: Success: {result.success}, Message: {result.message}')
        rclpy.shutdown()
