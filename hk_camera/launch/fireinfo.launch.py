from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, FindExecutable, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os
# It's good practice to import lifecycle_msgs if you are dealing with lifecycle states
import lifecycle_msgs.msg

def generate_launch_description():
    # Get the package share directory
    pkg_dir = get_package_share_directory('hk_camera')
    
    # Load config file
    config_file = os.path.join(pkg_dir, 'config', 'hk.yaml')
    
    node_name = 'fireinfo_pub' # Ensure this matches the node name in your C++ code

    camera_node = Node(
        package='hk_camera',
        executable='fireinfo_pub',
        name=node_name,
        parameters=[config_file],
        output='screen',
        emulate_tty=True
    )

    # Command to configure the node
    configure_cmd = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' lifecycle set ',
            node_name, # Corrected: use the variable
            ' configure'
        ]],
        shell=True,
        output='screen'
    )

    # Command to activate the node
    activate_cmd = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' lifecycle set ',
            node_name, # Corrected: use the variable
            ' activate'
        ]],
        shell=True,
        output='screen'
    )

    # Event handler to trigger configure when the node starts
    configure_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=camera_node,
            on_start=[configure_cmd]
        )
    )

    # Event handler to trigger activate when the configure command exits
    activate_event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=configure_cmd, # Target the configure_cmd directly
            on_exit=[activate_cmd]
        )
    )
    
    return LaunchDescription([
        camera_node,
        configure_event_handler,
        activate_event_handler
    ])