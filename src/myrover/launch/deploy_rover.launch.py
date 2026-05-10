from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 1. Launch Gazebo and the Robot
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("myrover"), "launch", "robot_gazebo.launch.py"]
            )
        )
    )

    # 2. Launch the Controllers (Delayed by 8 seconds to allow Gazebo to start)
    controllers_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("myrover"),
                            "launch",
                            "rover_controller.launch.py",
                        ]
                    )
                )
            )
        ],
    )

    # 3. Launch MoveIt MoveGroup (Delayed by 15 seconds to allow controllers to load and activate)
    move_group_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("roverv2_moveit_config"),
                            "launch",
                            "move_group.launch.py",
                        ]
                    )
                )
            )
        ],
    )

    return LaunchDescription(
        [
            gazebo_launch,
            controllers_launch,
            move_group_launch,
        ]
    )
