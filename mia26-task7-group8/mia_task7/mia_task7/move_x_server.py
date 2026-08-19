import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from turtlebot_interface.action import MoveX
import math
import time

class MoveXActionServer(Node):
    def __init__(self):
        super().__init__('move_x_action_server')
        
        self._action_server = ActionServer(
            self,
            MoveX,
            'move_x',
            self.execute_callback)
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.last_odom_time = self.get_clock().now()
        self.odom_received = False

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.last_odom_time = self.get_clock().now()
        self.odom_received = True

    def execute_callback(self, goal_handle):
        self.get_logger().info('Received goal to move forward...')
        
        if not self.odom_received:
            self.get_logger().error('Missing /odom data. Aborting.')
            goal_handle.abort()
            return MoveX.Result(success=False)

        start_x = self.current_x
        start_y = self.current_y
        target_dist = goal_handle.request.target_distance
        
        feedback_msg = MoveX.Feedback()
        vel_msg = Twist()
        vel_msg.linear.x = 0.2  
        distance_traveled = 0.0
        
        start_time = time.time()
        timeout_duration = 30.0

        while distance_traveled < target_dist:
            if (time.time() - start_time) > timeout_duration:
                self.get_logger().error('Hardware timeout. Aborting.')
                self.stop_robot()
                goal_handle.abort()
                return MoveX.Result(success=False)

            time_since_last_odom = (self.get_clock().now() - self.last_odom_time).nanoseconds / 1e9
            if time_since_last_odom > 2.0:
                self.get_logger().error('Lost /odom signal. Aborting.')
                self.stop_robot()
                goal_handle.abort()
                return MoveX.Result(success=False)

            self.cmd_vel_pub.publish(vel_msg)
            distance_traveled = math.sqrt((self.current_x - start_x)**2 + (self.current_y - start_y)**2)
            
            feedback_msg.current_distance = distance_traveled
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)

        self.stop_robot()
        goal_handle.succeed()
        
        result = MoveX.Result()
        result.success = True
        self.get_logger().info('Goal reached successfully!')
        return result

    def stop_robot(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    move_x_action_server = MoveXActionServer()
    rclpy.spin(move_x_action_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()