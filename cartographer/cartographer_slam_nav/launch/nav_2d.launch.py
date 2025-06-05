import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    pkg_share = FindPackageShare(package='cartographer_slam_nav').find('cartographer_slam_nav')
    
    #=====================参数配置=======================================================================
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    configuration_directory = LaunchConfiguration('configuration_directory',
                                                 default=os.path.join(pkg_share, 'param'))
    
    pbstream_file = LaunchConfiguration('pbstream_file', default='/home/cat/firebot_dragon/maps/my_map.pbstream')
    
    
    #=====================参数声明=======================================================================
    declare_mode_cmd = DeclareLaunchArgument(
        'mode',
        default_value='slam',
        description='Mode: slam or localization')
        
    declare_pbstream_cmd = DeclareLaunchArgument(
        'pbstream_file',
        default_value='',
        description='Path to pbstream file for localization mode')
        
    declare_config_basename_cmd = DeclareLaunchArgument(
        'configuration_basename',
        default_value='slam_2d.lua',
        description='Configuration file basename')


    #=====================定位模式的cartographer节点=================================================
    cartographer_localization_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', 'localization_2d.lua',
                   '-load_state_filename', pbstream_file])

    #=====================占用栅格节点================================================================
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])

    #===============================================启动文件========================================================
    ld = LaunchDescription()
    
    # 添加参数声明
    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_pbstream_cmd)
    ld.add_action(declare_config_basename_cmd)
    
    # 添加节点
    ld.add_action(cartographer_localization_node)
    ld.add_action(cartographer_occupancy_grid_node)

    return ld