#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 声明参数
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "board_width",
            default_value="9",
            description="Number of internal corners in width direction"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "board_height", 
            default_value="6",
            description="Number of internal corners in height direction"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "square_size",
            default_value="0.02",
            description="Size of checkerboard squares in meters"
        )
    )
    
    board_width = LaunchConfiguration("board_width")
    board_height = LaunchConfiguration("board_height") 
    square_size = LaunchConfiguration("square_size")

    # 启动机器人的MoveIt规划执行系统
    robot_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            FindPackageShare('robot_bringup').find('robot_bringup'),
            'launch', 'integrated_robot_bringup.launch.py')),
        launch_arguments={
            'gui': 'true',
            'use_camera': 'true',
            'use_gripper': 'true'
        }.items()
    )
    
    # 棋盘格检测节点
    checkerboard_detector = Node(
        package='robot_bringup',
        executable='checkerboard_detector.py',
        name='checkerboard_detector',
        parameters=[{
            'board_width': board_width,
            'board_height': board_height,
            'square_size': square_size,
            'camera_frame': 'camera_color_frame',
            'target_frame': 'calibration_target'
        }]
    )
    
    # 启动easy_handeye2标定
    handeye_calibration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            FindPackageShare('easy_handeye2').find('easy_handeye2'),
            'launch', 'calibrate.launch.py')),
        launch_arguments={
            'calibration_type': 'eye_in_hand',
            'name': 'rm75_eih_calib',
            
            # 机器人的tf框架参数
            'robot_base_frame': 'base_link',
            'robot_effector_frame': 'tool_link',
            
            # 跟踪系统的tf框架参数 (相机和标定板)
            'tracking_base_frame': 'camera_color_frame',
            'tracking_marker_frame': 'calibration_target',
            
            # 可选：禁用自动机器人移动，手动移动机器人
            'freehand_robot_movement': 'true'
        }.items()
    )

    return LaunchDescription(declared_arguments + [
        robot_moveit,
        checkerboard_detector,
        handeye_calibration
    ]) 