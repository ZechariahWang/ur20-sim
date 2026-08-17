from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")

    declared_args = [
        DeclareLaunchArgument("ur_type", default_value="ur20"),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="yyy.yyy.yyy.yyy",
            description="Ignored with fake hardware; set to URSim/robot IP otherwise.",
        ),
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
            description="MoveIt sends trajectories to this controller (its default).",
        ),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument(
            "launch_foxglove",
            default_value="true",
            description="Start foxglove_bridge (connect Foxglove Studio to ws://localhost:8765).",
        ),
    ]

    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake_hardware,
            "initial_joint_controller": initial_joint_controller,
            "launch_rviz": "false",
            # Wrapper description: fake hardware spawns in the real robot's
            # parked pose (config/initial_positions.yaml).
            "description_package": "ur20_sim",
            "description_file": "ur20_parked.urdf.xacro",
        }.items(),
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"]
            )
        ),
        launch_arguments={
            "ur_type": ur_type,
            "launch_rviz": launch_rviz,
        }.items(),
    )

    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        output="screen",
        parameters=[{"port": 8765}],
        condition=IfCondition(LaunchConfiguration("launch_foxglove")),
    )

    room_config = PathJoinSubstitution(
        [FindPackageShare("ur20_sim"), "config", "room.yaml"]
    )
    room_publisher = Node(
        package="ur20_sim",
        executable="room_publisher",
        output="screen",
        parameters=[room_config],
    )

    return LaunchDescription(declared_args + [ur_control, moveit, foxglove_bridge, room_publisher])
