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
            description="Size of the ArUco marker in meters"
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

    # 启动RealSense相机
    realsense_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            FindPackageShare('realsense2_camera').find('realsense2_camera'),
            'launch', 'rs_launch.py')),
        launch_arguments={
            'camera_name': 'camera',
            'camera_namespace': ''
        }.items()
    )
    
    # 我们的自定义ArUco检测节点
    aruco_detector_custom = Node(
        package='robot_bringup',
        executable='aruco_detector_node.py',
        name='aruco_detector_custom',
        parameters=[{
            'marker_id': marker_id,
            'marker_size': marker_size,
            'camera_frame': 'camera_color_frame',
            'marker_frame': 'aruco_marker_frame'
        }],
        output='screen'
    )

    return LaunchDescription(declared_arguments + [
        realsense_camera,
        aruco_detector_custom
    ]) 