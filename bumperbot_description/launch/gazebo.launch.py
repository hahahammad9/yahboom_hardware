#!/usr/bin/env python3
import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    bumperbot_description = get_package_share_directory("bumperbot_description")

    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(bumperbot_description, "urdf", "bumperbot.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )

    world_name_arg = DeclareLaunchArgument(
        name="world_name", 
        default_value="empty"
    )

    # Base install directory
    install_dir = str(Path(bumperbot_description).parent.resolve())
    
    # Path to the specific .world file
    world_path = PathJoinSubstitution([
        bumperbot_description,
        "worlds",
        PythonExpression(["'", LaunchConfiguration("world_name"), ".world'"])
    ])

    # Combine install root, the worlds folder, AND the models folder
    resource_paths = (
        install_dir + pathsep + 
        os.path.join(bumperbot_description, "worlds") + pathsep + 
        os.path.join(bumperbot_description, "models")
    )

    # Set both GZ and IGN environment variables for Humble compatibility
    gz_resource_path = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_paths)
    ign_resource_path = SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", resource_paths)

    ros_distro = os.environ.get("ROS_DISTRO", "humble")
    is_ignition = "true" if ros_distro == "humble" else "false"

    robot_description = ParameterValue(
        Command([
            "xacro ", LaunchConfiguration("model"),
            " is_ignition:=", is_ignition
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ]),
        launch_arguments={
            "gz_args": [world_path, " -v 4 -r"]
        }.items()
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "bumperbot"
        ],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"
        ],
        remappings=[
            ('/imu', '/imu/out'),
        ]
    )

    return LaunchDescription([
        model_arg,
        world_name_arg,
        gz_resource_path,
        ign_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
    ])