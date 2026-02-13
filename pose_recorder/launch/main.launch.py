#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # launch arguments
    robot_names_arg = DeclareLaunchArgument(
        'robot_names',
        default_value='neura_1;neura_2',
        description='Semicolon-separated list of robot names'
    )
    base_frames_arg = DeclareLaunchArgument(
        'base_frames',
        default_value='neura_1/world;neura_2/world',
        description='Semicolon-separated list of base frames'
    )
    ee_frames_arg = DeclareLaunchArgument(
        'ee_frames',
        default_value='neura_1/flange;neura_2/flange',
        description='Semicolon-separated list of end-effector frames'
    )
    save_dir_arg = DeclareLaunchArgument(
        'save_dir',
        default_value='/robotic_ws/src/pose_recorder/project_data/poses',
        description='Directory where poses are saved'
    )
    sequence_dir_arg = DeclareLaunchArgument(
        'sequence_dir',
        default_value='/robotic_ws/src/pose_recorder/project_data/sequences',
        description='Directory where sequences are saved'
    )
    relpose_dir_arg = DeclareLaunchArgument(
        'relpose_dir',
        default_value='/robotic_ws/src/pose_recorder/project_data/relative_poses',
        description='Directory where relative poses are saved'
    )
    config_dir_arg = DeclareLaunchArgument(
        'config_dir',
        default_value='/robotic_ws/src/pose_recorder/project_data/config',
        description='Directory where config files are saved'
    )

    # the GUI node
    gui_node = Node(
        package='pose_recorder',
        executable='main_window',
        name='pose_recorder_node',
        output='screen',
        parameters=[{
            'robot_names': LaunchConfiguration('robot_names'),
            'base_frames': LaunchConfiguration('base_frames'),
            'ee_frames':   LaunchConfiguration('ee_frames'),
            'save_dir':    LaunchConfiguration('save_dir'),
            'sequence_dir':LaunchConfiguration('sequence_dir'),
            'relpose_dir': LaunchConfiguration('relpose_dir'),
            'config_dir': LaunchConfiguration('config_dir')
        }],
    )

    return LaunchDescription([
        robot_names_arg,
        base_frames_arg,
        ee_frames_arg,
        save_dir_arg,
        sequence_dir_arg,
        relpose_dir_arg,
        config_dir_arg,
        gui_node
    ])
