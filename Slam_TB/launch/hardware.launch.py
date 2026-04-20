#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    bumperbot_description = get_package_share_directory("bumperbot_description")

    # Load the URDF model
    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(bumperbot_description, "urdf", "bumperbot.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str
    )

    # 1. Robot State Publisher (Publishes the physical TF tree)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": False}]
    )

    # 2. Custom Serial Bridge (Talks to MCU, publishes /odom)
    serial_bridge_node = Node(
        package="bumperbot_description",
        executable="serial_bridge.py",
        name="serial_bridge",
        output="screen"
    )

    # 3. SLLidar Node (Physical implementation of laser_link)
    rplidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'serial_port': '/dev/ydlidar', # Make sure this matches your Jetson's LiDAR port!
            'serial_baudrate': 115200,
            'frame_id': 'laser_link',
            'inverted': False,
            'angle_compensate': True,
        }],
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        serial_bridge_node,
        rplidar_node
    ])