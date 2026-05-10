import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Define the workspace root and specific file paths
    workspace_root = '/home/javideus/URJC/ThirdYear/modelingSimulations'
    model_path = os.path.join(workspace_root, 'practica3/src/myrover/robots/roverv2.xacro.urdf')
    rviz_config = os.path.join(workspace_root, 'practica3/src/myrover/rviz/view.rviz')
    
    # Get the directory of the urdf_visualizer launch file
    visualizer_launch_dir = os.path.join(get_package_share_directory('urdf_visualizer'), 'launch')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(visualizer_launch_dir, 'live.launch.py')),
            launch_arguments={
                'model': model_path,
                'rvizconfig': rviz_config
            }.items()
        )
    ])
