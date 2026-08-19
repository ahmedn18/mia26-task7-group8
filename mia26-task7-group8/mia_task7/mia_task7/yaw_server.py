import math
import rclpy

from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from robot_action.action import Move
from std_srvs.srv import SetBool


class yaw_server(Node):
    def __init__(self):
        super().__init__("yaw_server")

        self._action_server = ActionServer(self,Move,'/Move',self.execute_callback)

        # now make a function that subscribe with odmo to calculate the position of robot repeatly
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10 )

        # publish velocity to cmd_vel by using Twist
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_yaw=0.0

        # creating the service client to trigger opening walls on startup
        self.wall_client = self.create_client(SetBool, 'toggle_walls_1_2')
        self.call_wall_service_on_startup()

    # helper function to call the service immediately on startup
    def call_wall_service_on_startup(self):
        self.get_logger().info('Waiting for wall service...')
        self.wall_client.wait_for_service()
        request = SetBool.Request()
        request.data = True
        self.get_logger().info('Sending service request to open walls...')
        self.future = self.wall_client.call_async(request)

    # according to demo2 vid, i will make a callback function for odom
    def odom_callback(self, msg):

        orientation = msg.pose.pose.orientation

        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        # the next equation from ai
        self.current_yaw = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z)
        )
        
        #make a executa_callback function
    def execute_callback(self, goal):
        self.get_logger().info('Executing goal....')

        # turn_angle = goal.request.turn_angle
        first_yaw=goal.request.first_yaw
        second_yaw=goal.request.second_yaw

        feedback = Move.Feedback()
        result = Move.Result()

        #
        target_yaw = math.radians(first_yaw)

        while rclpy.ok():

            cmd = Twist()

            if self.current_yaw>=target_yaw:
                cmd.angular.z=0.0
                self.cmd_vel_publisher.publish(cmd)

                goal.succeed()
                result.success=True
                result.message=("robot reach target yaw")

                return result

            #trun robot
            cmd.angular.z=0.3
            self.cmd_vel_publisher.publish(cmd)

            feedback.current_action="turning_robot"
            goal.publish_feedback(feedback)

            rclpy.spin_once(self,timeout_sec=0.5)

def main():
    rclpy.init()

    yaw_server_instance = yaw_server()

    rclpy.spin(yaw_server_instance)

    yaw_server_instance.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
