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
        DeclareLaunchArgument("park_only", default_value="false"),
    ]

    robot_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare("ur20_sim"), "urdf", "ur20_parked.urdf.xacro"]
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
    main_yaml = PathJoinSubstitution([FindPackageShare("ur20_sim"), "config", "main_sweep.yaml"])
    room_yaml = PathJoinSubstitution([FindPackageShare("ur20_sim"), "config", "room.yaml"])

    main_sweep = Node(
        package="ur20_sim",
        executable="main_sweep",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"robot_description_semantic": robot_description_semantic},
            kinematics_yaml,
            room_yaml,
            main_yaml,
            {"park_only": ParameterValue(LaunchConfiguration("park_only"), value_type=bool)},
        ],
    )

    return LaunchDescription(declared_args + [main_sweep])
