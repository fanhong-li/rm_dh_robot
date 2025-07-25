import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_gripper",
            default_value="true",
            description="Enable gripper",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_camera",
            default_value="true",
            description="Enable RealSense camera",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 and joint state publisher GUI",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_gripper",
            default_value="false",
            description="Use mock gripper for testing without hardware",
        )
    )

    use_gripper = LaunchConfiguration("use_gripper")
    use_camera = LaunchConfiguration("use_camera")
    gui = LaunchConfiguration("gui")
    use_mock_gripper = LaunchConfiguration("use_mock_gripper")

    # 机械臂驱动
    rm_75_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rm_driver'),
            'launch', 'rm_75_driver.launch.py'))
    )

    # 机械臂控制
    rm_75_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rm_control'),
            'launch', 'rm_75_control.launch.py'))
    )

    # MoveIt2配置 - 使用集成的配置
    integrated_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('robot_bringup'),
            'launch', 'integrated_moveit.launch.py'))
    )

    # 夹爪 - 使用修复的配置
    dh_ag95_gripper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('dh_gripper_driver'),
            'launch', 'dh_ag95_gripper.launch.py')),
        launch_arguments={
            'use_mock_hardware': use_mock_gripper,
            'com_port': '/dev/robot/dh_ag95_gripper'
        }.items(),
        condition=IfCondition(use_gripper)
    )

    # RealSense相机
    realsense_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('realsense2_camera'),
            'launch', 'rs_launch.py')),
        condition=IfCondition(use_camera)
    )

    # joint_states合并节点
    joint_states_merger = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('robot_bringup'),
            'launch', 'joint_states_merger.launch.py'))
    )

    # 机器人描述与可视化
    robot_description_display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('robot_description'),
            'launch', 'display.launch.py')),
        launch_arguments={
            'use_gripper': use_gripper,
            'use_camera': use_camera,
            'gui': gui
        }.items()
    )

    return LaunchDescription(declared_arguments + [
        rm_75_driver,
        rm_75_control,
        integrated_moveit,
        dh_ag95_gripper,
        realsense_camera,
        joint_states_merger,
        robot_description_display
    ]) 