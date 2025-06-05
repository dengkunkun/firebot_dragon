#!/usr/bin/env python3
# filepath: /home/cat/firebot_dragon/src/cartographer/cartographer_slam_nav/launch/nav2_cartographer_bringup.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushROSNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # 获取包路径
    bringup_dir = get_package_share_directory('cartographer_slam_nav')
    launch_dir = os.path.join(bringup_dir, 'launch')
    
    # 启动配置变量
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    
    # Cartographer相关参数
    mode = LaunchConfiguration('mode')
    pbstream_file = LaunchConfiguration('pbstream_file')
    configuration_directory = LaunchConfiguration('configuration_directory')
    configuration_basename = LaunchConfiguration('configuration_basename')
    resolution = LaunchConfiguration('resolution')
    publish_period_sec = LaunchConfiguration('publish_period_sec')

    # TF重映射
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # 参数文件配置
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # 环境变量
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    # ===================== 参数声明 =====================
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', 
        default_value='', 
        description='Top-level namespace'
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'param', 'nav2_2d.yaml'),
        description='Full path to the ROS2 parameters file to use'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Whether to use composed bringup'
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes'
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', 
        default_value='info', 
        description='log level'
    )

    # Cartographer相关参数
    declare_mode_cmd = DeclareLaunchArgument(
        'mode',
        default_value='localization',
        description='Cartographer mode: slam or localization'
    )

    declare_pbstream_cmd = DeclareLaunchArgument(
        'pbstream_file',
        default_value='',
        description='Path to pbstream file for localization mode'
    )

    declare_configuration_directory_cmd = DeclareLaunchArgument(
        'configuration_directory',
        default_value=os.path.join(bringup_dir, 'param'),
        description='Cartographer configuration directory'
    )

    declare_configuration_basename_cmd = DeclareLaunchArgument(
        'configuration_basename',
        default_value='localization_2d.lua',
        description='Cartographer configuration file basename'
    )

    declare_resolution_cmd = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='Map resolution'
    )

    declare_publish_period_cmd = DeclareLaunchArgument(
        'publish_period_sec',
        default_value='1.0',
        description='Map publish period'
    )

    # ===================== 节点定义 =====================
    
    # Cartographer SLAM节点
    cartographer_slam_node = Node(
        condition=IfCondition(PythonExpression(["'", mode, "' == 'slam'"])),
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', configuration_directory,
            '-configuration_basename', 'slam_2d.lua'
        ],
        remappings=remappings
    )

    # Cartographer定位节点
    cartographer_localization_node = Node(
        condition=IfCondition(PythonExpression(["'", mode, "' == 'localization'"])),
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', configuration_directory,
            '-configuration_basename', configuration_basename,
            '-load_state_filename', pbstream_file
        ],
        remappings=remappings
    )

    # Cartographer占用栅格节点
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-resolution', resolution, 
            '-publish_period_sec', publish_period_sec
        ],
        remappings=remappings
    )

    # Controller Server
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        respawn=LaunchConfiguration('use_respawn'),
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]
    )

    # Smoother Server
    smoother_server_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        respawn=LaunchConfiguration('use_respawn'),
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings
    )

    # Planner Server
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        respawn=LaunchConfiguration('use_respawn'),
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings
    )

    # Behavior Server
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        respawn=LaunchConfiguration('use_respawn'),
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]
    )

    # BT Navigator
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        respawn=LaunchConfiguration('use_respawn'),
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings
    )

    # Waypoint Follower
    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        respawn=LaunchConfiguration('use_respawn'),
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings
    )

    # Velocity Smoother
    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        respawn=LaunchConfiguration('use_respawn'),
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]
    )

    # Collision Monitor
    # collision_monitor_node = Node(
    #     package='nav2_collision_monitor',
    #     executable='collision_monitor',
    #     name='collision_monitor',
    #     output='screen',
    #     respawn=LaunchConfiguration('use_respawn'),
    #     respawn_delay=2.0,
    #     parameters=[configured_params],
    #     arguments=['--ros-args', '--log-level', log_level],
    #     remappings=remappings
    # )

    # Lifecycle Manager
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'autostart': autostart,
            'node_names': [
                'controller_server',
                'smoother_server', 
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
                # 'collision_monitor'
            ]
        }]
    )

    # ===================== 组合所有节点 =====================
    bringup_cmd_group = GroupAction([
        PushROSNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace
        ),
        cartographer_slam_node,
        cartographer_localization_node,
        cartographer_occupancy_grid_node,
        controller_server_node,
        smoother_server_node,
        planner_server_node,
        behavior_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        velocity_smoother_node,
        # collision_monitor_node,
        lifecycle_manager_node,
    ])

    # ===================== 创建LaunchDescription =====================
    ld = LaunchDescription()

    # 环境变量
    ld.add_action(stdout_linebuf_envvar)

    # 参数声明
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_pbstream_cmd)
    ld.add_action(declare_configuration_directory_cmd)
    ld.add_action(declare_configuration_basename_cmd)
    ld.add_action(declare_resolution_cmd)
    ld.add_action(declare_publish_period_cmd)

    # 添加节点组
    ld.add_action(bringup_cmd_group)

    return ld