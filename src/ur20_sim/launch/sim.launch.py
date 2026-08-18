from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
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
        DeclareLaunchArgument(
            "launch_webapp",
            default_value="true",
            description="Start rosbridge and the command server for the webapp.",
        ),
        DeclareLaunchArgument("rosbridge_port", default_value="9090"),
    ]

    # Webapp interface: rosbridge websocket plus the command server that
    # runs/aborts the sweep routines on webapp commands.
    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("rosbridge_server"), "launch", "rosbridge_websocket_launch.xml"]
            )
        ),
        launch_arguments={"port": LaunchConfiguration("rosbridge_port")}.items(),
        condition=IfCondition(LaunchConfiguration("launch_webapp")),
    )
    command_server = Node(
        package="ur20_sim",
        executable="command_server",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_webapp")),
    )

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
            # Factory calibration extracted from the real robot 2026-08-17.
            "kinematics_params_file": PathJoinSubstitution(
                [FindPackageShare("ur20_sim"), "config", "ur20_calibration.yaml"]
            ),
            # We run our own dashboard client below with respawn, because
            # the driver's copy gives up after 2 s if the robot is still
            # booting when the stack launches.
            "launch_dashboard_client": "false",
        }.items(),
    )

    # Same node the driver would start, but with respawn: if the robot's
    # dashboard server is not up yet (robot still booting), the client
    # exits and simply retries until it connects.
    dashboard_client = Node(
        package="ur_robot_driver",
        executable="dashboard_client",
        name="dashboard_client",
        output="screen",
        emulate_tty=True,
        respawn=True,
        respawn_delay=3.0,
        parameters=[{"robot_ip": robot_ip}],
        condition=UnlessCondition(use_fake_hardware),
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
            # Same wrapper description as the driver so move_group's
            # planning model includes the camera for collision checking.
            "description_package": "ur20_sim",
            "description_file": "ur20_parked.urdf.xacro",
        }.items(),
    )

    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        output="screen",
        parameters=[{
            "port": 8765,
            # No parameter browsing: we configure everything via YAML, and
            # the bridge's bulk parameter queries time out on busy nodes
            # and spam errors. Topics, services, and the 3D view keep working.
            "capabilities": ["clientPublish", "services", "connectionGraph", "assets"],
        }],
        condition=IfCondition(LaunchConfiguration("launch_foxglove")),
    )

    disable_friction_controller = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "for i in $(seq 1 30); do "
            "ros2 control list_controllers 2>/dev/null | grep friction_model_controller | grep -q inactive && "
            "{ echo 'friction_model_controller is inactive'; exit 0; }; "
            "ros2 control set_controller_state friction_model_controller inactive 2>/dev/null && exit 0; "
            "sleep 2; done; "
            "echo 'could not deactivate friction_model_controller'",
        ],
        output="screen",
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

    return LaunchDescription(declared_args + [ur_control, dashboard_client, moveit, foxglove_bridge, room_publisher, disable_friction_controller, rosbridge, command_server])
