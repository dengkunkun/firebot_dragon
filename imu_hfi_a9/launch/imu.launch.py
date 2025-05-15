from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Get the package share directory
    pkg_dir = get_package_share_directory('imu_hfi_a9')
    
    # Load config file
    config_file = os.path.join(pkg_dir, 'config', 'config.yaml')
    
    # Create IMU node
    imu_node = Node(
        package='imu_hfi_a9',
        executable='imu_hfi_a9_node',
        name='imu_hfi_a9',
        parameters=[config_file],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        imu_node
    ])