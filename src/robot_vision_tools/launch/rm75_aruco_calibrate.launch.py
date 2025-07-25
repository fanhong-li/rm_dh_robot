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
            "marker_size",
            default_value="0.1",
            description="Size of the ArUco marker in meters (e.g., 0.1 for 10cm)"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "marker_id",
            default_value="0",
            description="ID of the ArUco marker"
        )
    )
    
    marker_size = LaunchConfiguration("marker_size")
    marker_id = LaunchConfiguration("marker_id")

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
    
    # ArUco检测节点 - 使用正确的相机话题
    aruco_detector = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single',
        parameters=[{
            'image_transport': 'raw',
            'publish_tf': True,
            'marker_size': marker_size,
            'marker_id': marker_id,
            'reference_frame': 'camera_color_frame',
            'camera_frame': 'camera_color_frame',
            'marker_frame': 'aruco_marker_frame',
            'corner_refinement': 'LINES'
        }],
        remappings=[
            ('/camera_info', '/camera/color/camera_info'),
            ('/image', '/camera/color/image_raw')
        ]
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
            
            # 跟踪系统的tf框架参数 (相机和ArUco标记)
            'tracking_base_frame': 'camera_color_frame',
            'tracking_marker_frame': 'aruco_marker_frame',
            
            # 可选：禁用自动机器人移动，手动移动机器人
            'freehand_robot_movement': 'true'
        }.items()
    )

    return LaunchDescription(declared_arguments + [
        robot_moveit,
        aruco_detector,
        handeye_calibration
    ]) 