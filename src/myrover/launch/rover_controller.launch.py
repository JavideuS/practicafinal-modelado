from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument
from controller_manager.launch_utils import generate_load_controller_launch_description


def generate_launch_description():
    declare_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="use_sim_time simulation parameter",
    )
    pkg_share_folder = get_package_share_directory("myrover")

    # Load joint state broadcaster controller
    joint_state_broadcaster = GroupAction(
        [
            generate_load_controller_launch_description(
                controller_name="joint_state_broadcaster",
                controller_params_file=join(
                    pkg_share_folder, "config", "roverv2_controllers.yaml"
                ),
            )
        ],
    )
    # Load rover controller
    base_controller = GroupAction(
        [
            generate_load_controller_launch_description(
                controller_name="rover_diff_drive_controller",
                controller_params_file=join(
                    pkg_share_folder, "config", "roverv2_controllers.yaml"
                ),
            )
        ],
    )

    # Arm and end-effector controllers
    arm_pkg_share_folder = get_package_share_directory("myrover")
    # Load arm controller
    arm_controller = GroupAction(
        [
            generate_load_controller_launch_description(
                controller_name="scara_controller",
                controller_params_file=join(
                    arm_pkg_share_folder, "config", "roverv2_controllers.yaml"
                ),
            )
        ],
    )
    # Load gripper controller
    gripper_controller = GroupAction(
        [
            generate_load_controller_launch_description(
                controller_name="gripper_controller",
                controller_params_file=join(
                    arm_pkg_share_folder, "config", "roverv2_controllers.yaml"
                ),
            )
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_sim_time)
    ld.add_action(joint_state_broadcaster)
    ld.add_action(base_controller)
    ld.add_action(arm_controller)
    ld.add_action(gripper_controller)

    return ld
