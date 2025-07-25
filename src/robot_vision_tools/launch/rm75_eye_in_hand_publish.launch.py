#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
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
    
    # 启动跟踪系统的ROS驱动 (RealSense相机已包含在上面)
    
    # 发布手眼标定结果
    handeye_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            FindPackageShare('easy_handeye2').find('easy_handeye2'),
            'launch', 'publish.launch.py')),
        launch_arguments={
            'name': 'rm75_eih_calib'  # 使用与标定时相同的名称
        }.items()
    )

    return LaunchDescription([
        robot_moveit,
        handeye_publisher
    ]) 