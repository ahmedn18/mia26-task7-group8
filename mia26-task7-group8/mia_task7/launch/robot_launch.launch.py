from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ultrasonic_node = Node(
        package='mia_task7',
        executable='yaw_server',
        name='yaw_server'
  
    )
    controller_node = Node(
        package='mia_task7',
        executable='move_yaw_client',
        name='move_yaw_client'
      
    )
    return LaunchDescription([ultrasonic_node, controller_node])