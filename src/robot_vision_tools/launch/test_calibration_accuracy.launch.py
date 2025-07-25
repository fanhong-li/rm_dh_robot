#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 启动机器人系统（使用URDF中的估计相机位置）
    robot_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            FindPackageShare('robot_bringup').find('robot_bringup'),
            'launch', 'integrated_robot_bringup.launch.py')),
        launch_arguments={
            'gui': 'true',
            'use_camera': 'true',
            'use_gripper': 'true'
        }.items()
    )
    
    # 发布标定结果（精确的相机位置）
    handeye_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            FindPackageShare('easy_handeye2').find('easy_handeye2'),
            'launch', 'publish.launch.py')),
        launch_arguments={
            'name': 'rm75_eih_calib'
        }.items()
    )
    
    # 启动我们的ArUco检测节点
    aruco_detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            FindPackageShare('robot_bringup').find('robot_bringup'),
            'launch', 'test_aruco_custom.launch.py')),
        launch_arguments={
            'marker_id': '0',
            'marker_size': '0.1'
        }.items()
    )

    return LaunchDescription([
        robot_system,
        handeye_publisher,
        aruco_detector
    ]) 