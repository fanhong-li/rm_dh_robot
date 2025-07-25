#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 只启动RQT标定节点，用于测试
    handeye_rqt_calibrator = Node(
        package='easy_handeye2', 
        executable='rqt_calibrator.py',
        name='handeye_rqt_calibrator',
        arguments=['--ros-args', '--log-level', 'debug'],
        parameters=[{
            'name': 'rm75_eih_calib',
            'calibration_type': 'eye_in_hand',
            'tracking_base_frame': 'camera_color_frame',
            'tracking_marker_frame': 'aruco_marker_frame',
            'robot_base_frame': 'base_link',
            'robot_effector_frame': 'tool_link',
        }],
        output='screen'
    )

    return LaunchDescription([
        handeye_rqt_calibrator
    ]) 