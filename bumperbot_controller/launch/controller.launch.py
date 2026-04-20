#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--switch-timeout", "600",
            "--controller-manager-timeout", "600",
            "--service-call-timeout", "600",
        ],
    )

    ackermann_steering_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ackermann_steering_controller",
            "--controller-manager", "/controller_manager",
            "--switch-timeout", "600",
            "--controller-manager-timeout", "600",
            "--service-call-timeout", "600",
        ],
    )

    # Delay ackermann controller until joint_state_broadcaster is active
    delay_ackermann_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[ackermann_steering_controller_spawner],
        )
    )

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        delay_ackermann_after_jsb,
    ])