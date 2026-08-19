"""Launch the maze, both action servers and the solver, in one shot.

The run timer starts when the simulation launches, not when the solver does,
so everything comes up from a single command:

    ros2 launch mia_task7 solve_maze.launch.py

Set launch_simulation:=false to attach the solver to a maze that is already
running -- useful while debugging a single stage.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    gate_service = LaunchConfiguration('gate_service')
    launch_simulation = LaunchConfiguration('launch_simulation')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use the /clock published by Gazebo. Keep this true.')

    declare_gate_service = DeclareLaunchArgument(
        'gate_service',
        default_value='toggle_walls_1_2',
        description='SetBool service that moves the red gates.')

    declare_launch_simulation = DeclareLaunchArgument(
        'launch_simulation',
        default_value='true',
        description='Start the maze too. Set false to attach to a running one.')

    maze_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('maze_control'),
                'launch',
                'maze_simulation_tb3.launch.py')),
        condition=IfCondition(launch_simulation))

    move_x_server = Node(
        package='mia_task7',
        executable='move_x_server',
        name='move_x_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])

    move_yaw_server = Node(
        package='mia_task7',
        executable='move_yaw_server',
        name='move_yaw_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])

    solve_maze = Node(
        package='mia_task7',
        executable='solve_maze',
        name='solve_maze',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'gate_service': gate_service,
        }])

    return LaunchDescription([
        declare_use_sim_time,
        declare_gate_service,
        declare_launch_simulation,
        maze_simulation,
        move_x_server,
        move_yaw_server,
        solve_maze,
    ])
