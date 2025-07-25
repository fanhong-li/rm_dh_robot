import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_gripper",
            default_value="true",
            description="Enable gripper in robot description",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_camera",
            default_value="true",
            description="Enable camera in robot description",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 and joint state publisher GUI",
        )
    )

    # Initialize arguments
    use_gripper = LaunchConfiguration("use_gripper")
    use_camera = LaunchConfiguration("use_camera")
    gui = LaunchConfiguration("gui")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("robot_description"),
                    "urdf",
                    "integrated_robot.urdf.xacro",
                ]
            ),
            " ",
            "use_gripper:=",
            use_gripper,
            " ",
            "use_camera:=",
            use_camera,
        ]
    )
    # Robot state publisher node
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description_content}],
        remappings=[
            ('joint_states', '/robot_state_publisher/joint_states')
        ]
    )

    # Joint state publisher node - 禁用，因为机械臂和夹爪已经在发布真实的关节状态
    # joint_state_publisher_node = Node(
    #     package="joint_state_publisher",
    #     executable="joint_state_publisher",
    #     name="joint_state_publisher",
    # )

    # Joint state publisher GUI node
    # joint_state_publisher_gui_node = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui",
    #     name="joint_state_publisher_gui",
    #     condition=IfCondition(gui),
    # )

    # RViz node
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("robot_description"), "rviz", "integrated_robot.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    nodes = [
        robot_state_pub_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes) 