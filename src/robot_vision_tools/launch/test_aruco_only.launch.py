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

    # 只启动RealSense相机
    realsense_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            FindPackageShare('realsense2_camera').find('realsense2_camera'),
            'launch', 'rs_launch.py')),
        launch_arguments={
            'camera_name': 'camera',
            'camera_namespace': ''
        }.items()
    )
    
    # ArUco检测节点
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
            ('/camera_info', '/camera/camera/color/camera_info'),
            ('/image', '/camera/camera/color/image_raw')
        ],
        output='screen'
    )

    return LaunchDescription(declared_arguments + [
        realsense_camera,
        aruco_detector
    ]) 