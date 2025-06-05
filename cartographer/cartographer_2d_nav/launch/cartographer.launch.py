# home/cat/firebot_dragon/src/navigation/launch/nav2_cartographer.launch.py

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    # cartographer_config_dir = LaunchConfiguration('cartographer_config_dir')
    # configuration_basename = LaunchConfiguration('configuration_basename')
    nav2_params_file = LaunchConfiguration('params_file')

    # 参数声明
    # declare_use_sim_time_cmd = DeclareLaunchArgument(
    #     'use_sim_time',
    #     default_value='false',
    #     description='Use simulation (Gazebo) clock if true')

    # declare_map_yaml_cmd = DeclareLaunchArgument(
    #     'map',
    #     default_value=os.path.join(get_package_share_directory('navigation'), 'maps', 'my_map.yaml'),
    #     description='Full path to map yaml file to load')

    # declare_cartographer_config_dir_cmd = DeclareLaunchArgument(
    #     'cartographer_config_dir',
    #     default_value=os.path.join(get_package_share_directory('navigation'), 'config'),
    #     description='Full path to config file to load')

    # declare_configuration_basename_cmd = DeclareLaunchArgument(
    #     'configuration_basename',
    #     default_value='cartographer_localization.lua',
    #     description='Name of cartographer config file')

    # declare_params_file_cmd = DeclareLaunchArgument(
    #     'params_file',
    #     default_value=os.path.join(get_package_share_directory('navigation'), 'config', 'nav2_params_cartographer.yaml'),
    #     description='Full path to param file to load')

    # # 启动cartographer定位
    # cartographer_node = Node(
    #     package='cartographer_ros',
    #     executable='cartographer_node',
    #     name='cartographer_node',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     arguments=['-configuration_directory', cartographer_config_dir,
    #                '-configuration_basename', configuration_basename,
    #                '-load_state_filename', LaunchConfiguration('map_file', default='')],
    #     remappings=[
    #         ('scan', '/scan'),
    #         ('odom', '/odom'),
    #         ('imu', '/imu/data')
    #     ])

    # # 启动occupancy grid节点
    # occupancy_grid_node = Node(
    #     package='cartographer_ros',
    #     executable='cartographer_occupancy_grid_node',
    #     name='cartographer_occupancy_grid_node',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     arguments=['-resolution', '0.05'])

    # 启动Nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'map_subscribe_transient_local': 'true'
        }.items()
    )

    # 地图服务器（如果需要预加载地图）
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_file
        }])

    # 生命周期管理器
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server']}])

    return LaunchDescription([
        # declare_use_sim_time_cmd,
        # declare_map_yaml_cmd,
        # declare_cartographer_config_dir_cmd,
        # declare_configuration_basename_cmd,
        # declare_params_file_cmd,

        # cartographer_node,
        # occupancy_grid_node,
        map_server_node,
        lifecycle_manager_node,
        nav2_launch
    ])