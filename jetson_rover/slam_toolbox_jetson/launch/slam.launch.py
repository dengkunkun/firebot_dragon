from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
#这个默认无法启动，需要手动配置和激活生命周期，可能是新版改为默认使用生命周期了
def generate_launch_description():
    # Get parameters file path
    pkg_dir = get_package_share_directory('slam_toolbox_jetson')
    params_file = os.path.join(pkg_dir, 'config', 'slam_params.yaml')
    
    # Configure SLAM Toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',  # Changed to sync version
        name='slam_toolbox',
        output='screen',
        parameters=[{'params_file': params_file}],  # Pass params file directly
        remappings=[
            ('scan', 'scan'),
            ('tf', 'tf'),
            ('tf_static', 'tf_static'),
            ('map', 'map'),
            ('map_metadata', 'map_metadata')
        ]
    )

    return LaunchDescription([
        slam_toolbox_node
    ])