# ============================================================
# Imports
# ============================================================
from os.path import join
from os import environ, pathsep

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


# ============================================================
# Helper: launch Gazebo server and GUI client
# ============================================================
def start_gzserver(context, *args, **kwargs):
    pkg_path = get_package_share_directory("urjc_excavation_world")
    world_name = LaunchConfiguration("world_name").perform(context)
    world = join(pkg_path, "worlds", world_name + ".world")

    # Gazebo server (-s = server only, -r = run immediately)
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": ["-r -s -v 4 ", world]}.items(),
    )

    # Gazebo GUI client
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": [" -g "]}.items(),
    )

    return [start_gazebo_server_cmd, start_gazebo_client_cmd]


# ============================================================
# Helper: build GZ_SIM_RESOURCE_PATH from installed packages
# ============================================================
def get_model_paths(packages_names):
    model_paths = ""
    for package_name in packages_names:
        if model_paths != "":
            model_paths += pathsep
        package_path = get_package_prefix(package_name)
        model_path = join(package_path, "share")
        model_paths += model_path

    if "GZ_SIM_RESOURCE_PATH" in environ:
        model_paths += pathsep + environ["GZ_SIM_RESOURCE_PATH"]

    return model_paths


# ============================================================
# Main launch description
# ============================================================
def generate_launch_description():

    # --- MoveIt config (used by move_group and RViz) ---
    # To configure elements in moveIt, not necessary for now
    moveit_config = MoveItConfigsBuilder(
        "roverv2", package_name="roverv2_moveit_config"
    ).to_moveit_configs()

    # --- Package paths ---
    pkg_path = get_package_share_directory("myrover")

    # --- Model/resource paths for Gazebo ---
    model_path = get_model_paths(["myrover"])

    # ── Launch arguments ─────────────────────────────────────
    declare_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock",
    )

    declare_world_name = DeclareLaunchArgument(
        "world_name",
        default_value="urjc_excavation_msr",
        description="Name of the .world file (without extension) inside urjc_excavation_world/worlds/",
    )

    # ── Gazebo (server + client via OpaqueFunction) ───────────
    start_gazebo_server_cmd = OpaqueFunction(function=start_gzserver)

    # ── Spawn robot into Gazebo ───────────────────────────────
    gazebo_spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-model",
            "roverv2",
            "-topic",
            "robot_description",
            "-x",
            "-5",
            "-y",
            "0",
            "-z",
            "-5.5",
            "-use_sim_time",
            "True",
        ],
    )

    # ── Robot State Publisher (publishes /robot_description) ──
    robot_description_launcher = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("roverv2_moveit_config"), "launch", "rsp.launch.py"]
        )
    )

    # Removed joint_state_publisher_gui as it conflicts with Gazebo's joint_state_broadcaster

    # ── RViz2 ─────────────────────────────────────────────────
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("myrover"), "rviz", "roverv2.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
    )

    # ── Gazebo ↔ ROS 2 topic bridge ───────────────────────────
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bridge_ros_gz",
        parameters=[
            {
                "config_file": join(pkg_path, "config", "roverv2_bridge.yaml"),
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    # ── Camera image bridge (optimised for image messages) ────
    gz_image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/cam_front/image", "/cam_scara/image"],
        output="screen",
        parameters=[
            {"use_sim_time": True, "camera.image.compressed.jpeg_quality": 75},
        ],
    )

    diff_drive_controller_name = "rover_diff_drive_controller"

    # ── Twist stamper (adds timestamp required by diff-drive) ─
    twist_stamped = Node(
        package="twist_stamper",
        executable="twist_stamper",
        name="twist_stamper",
        output="screen",
        parameters=[{"use_sim_time": True}],
        remappings=[
            ("cmd_vel_out", f"/{diff_drive_controller_name}/cmd_vel"),
            ("cmd_vel_in", "/cmd_vel"),
        ],
    )

    # ── Assemble LaunchDescription ────────────────────────────
    ld = LaunchDescription()

    # Arguments first
    ld.add_action(declare_sim_time)
    ld.add_action(declare_world_name)

    # Environment variables for Gazebo model discovery
    ld.add_action(SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", model_path))
    ld.add_action(SetEnvironmentVariable("GZ_SIM_MODEL_PATH", model_path))

    # Robot description must be up before spawning
    ld.add_action(robot_description_launcher)

    # Gazebo (world + GUI)
    ld.add_action(start_gazebo_server_cmd)

    # Spawn robot after Gazebo is ready
    ld.add_action(gazebo_spawn_robot)

    # Bridges (need Gazebo + robot topics to exist)
    ld.add_action(bridge)
    ld.add_action(gz_image_bridge_node)

    # MoveIt: move_group and controllers
    # ld.add_action(move_group_launcher)
    # ld.add_action(spawn_controllers_launcher)

    # Extras
    ld.add_action(twist_stamped)
    ld.add_action(rviz_node)

    return ld
