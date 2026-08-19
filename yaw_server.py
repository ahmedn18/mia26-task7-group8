<<<<<<< HEAD
# now we need to recieve order from odmo to play yaw_client

import rclpy
import rclpy.node import Node
=======
# now we need to recieve order from odmo to play our goal

import math
import rclpy

from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from robot_action.action import Move

class yaw_server(Node):
    def __init__(self):
        super().__init__("yaw_server")

        self._action_server = ActionServer(self,Move,'/Move',self.execute_callback)

        # now make a function that subscribe with odmo to calculate the position of robot repeatly
        self.odom_subscriber = self.create_subscription(Odometry, '/Odom', self.odom_callback, 10 )

        # publish velocity to cmd_vel by using Twist
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_yaw=0.0

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

            #trun robot

            cmd.angular.z=0.3
            self.cmd_vel_publisher.publish(cmd)

            feedback.current_action="turning_robot"
            goal.publish_feedback(feedback)

            rclpy.spin(self,time=0.5)

            goal.succeed()

            result.success=True
            result.message=("robot reach target yaw")

            return result

def main():
    rclpy.init()

    yaw_server = yaw_server()

    rclpy.spin(yaw_server)

    yaw_server.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

















    




>>>>>>> 0384d61 (add yaw_server)
