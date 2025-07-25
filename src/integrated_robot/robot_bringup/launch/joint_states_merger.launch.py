from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    merger = Node(
        package="robot_bringup",
        executable="joint_states_merger.py",
        name="joint_states_merger",
        output="screen",
        parameters=[],
        arguments=[],
        prefix="python3 "
    )
    return LaunchDescription([merger]) 