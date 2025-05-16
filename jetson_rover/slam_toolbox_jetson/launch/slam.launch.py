from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    base_frame = LaunchConfiguration('base_frame')
    
    # 获取默认参数文件路径
    slam_params_file = os.path.join(
        get_package_share_directory('slam_toolbox_jetson'),
        'config',
        'slam_params.yaml'
    )

    # 配置 SLAM Toolbox 节点
    slam_toolbox_node = Node(
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time,
             'base_frame': base_frame,
             'odom_frame': 'odom',
             'map_frame': 'map',
             'transform_publish_period': 0.05,
             'map_update_interval': 5.0}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        remappings=[
            ('/scan', 'scan'),  # Ensure this matches your laser scan topic
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/map', 'map'),
            ('/map_metadata', 'map_metadata')
        ]
    )

    return LaunchDescription([
        # 声明 launch 参数
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Base frame id'
        ),
        slam_toolbox_node
    ])