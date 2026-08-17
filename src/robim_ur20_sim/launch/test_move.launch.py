"""Run the test_move node: a small wrist wiggle to verify real-robot control."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")

    declared_args = [
        DeclareLaunchArgument("ur_type", default_value="ur20"),
    ]

    robot_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]
                ),
                " ",
                "name:=ur",
                " ",
                "ur_type:=",
                ur_type,
            ]
        ),
        value_type=str,
    )

    robot_description_semantic = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]
                ),
                " ",
                "name:=ur",
            ]
        ),
        value_type=str,
    )

    kinematics_yaml = PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "config", "kinematics.yaml"])

    test_move = Node(
        package="robim_ur20_sim",
        executable="test_move",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"robot_description_semantic": robot_description_semantic},
            kinematics_yaml,
        ],
    )

    return LaunchDescription(declared_args + [test_move])
