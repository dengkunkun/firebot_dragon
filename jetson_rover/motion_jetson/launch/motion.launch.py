from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Get the package share directory
    pkg_dir = get_package_share_directory('motion_jetson')
    
    # Load config file
    config_file = os.path.join(pkg_dir, 'config', 'param.yaml')
    
    # Create IMU node
    motion_node = Node(
        package='motion_jetson',
        executable='motion_jetson',
        name='motion_jetson',
        parameters=[config_file],
        output='screen',    # 输出到屏幕
        emulate_tty=True, #保持 ANSI 颜色输出
    )
    
    return LaunchDescription([
        motion_node
    ])