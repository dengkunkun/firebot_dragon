import os,sys
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import PushRosNamespace
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction, TimerAction
from pathlib import Path

scripts_dir = os.path.join(get_package_share_directory('nav2_jetson'), 'scripts')
sys.path.insert(0, scripts_dir)
try:
    from yaml_join import merge_yaml_files # Or specific functions you need
except ImportError as e:
    print(f"Error importing yaml_join: {e}. Ensure yaml_join.py is in the Python path (e.g., nav2_jetson/scripts).")
    merge_yaml_files = None # Fallback or error indicator
    raise ImportError(f"Failed to import yaml_join: {e}")


def launch_setup(context):
    navigation_package_path = get_package_share_directory('nav2_jetson')
    config_dir = os.path.join(navigation_package_path, 'config') # Standard config directory

    maps_path = str(Path.home() / 'firebot_dragon' / 'maps')
    sim = LaunchConfiguration('sim', default='false').perform(context)
    map_name = LaunchConfiguration('map', default='map_office').perform(context)
    robot_name = LaunchConfiguration('robot_name', default='/').perform(context)
    master_name = LaunchConfiguration('master_name', default='/').perform(context)
    use_teb = LaunchConfiguration('use_teb', default='false').perform(context)
    use_stvl_str = LaunchConfiguration('use_stvl', default='false').perform(context)

    sim_arg = DeclareLaunchArgument('sim', default_value=sim)
    map_name_arg = DeclareLaunchArgument('map', default_value=map_name)
    master_name_arg = DeclareLaunchArgument('master_name', default_value=master_name)
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value=robot_name)
    use_teb_arg = DeclareLaunchArgument('use_teb', default_value=use_teb)

    use_sim_time = 'true' if sim == 'true' else 'false'
    use_namespace = 'true' if robot_name != '/' else 'false'
    
    base_nav2_params_file = os.path.join(config_dir, "nav2_params_base.yaml") 
        
    if use_stvl_str.lower() == 'true':
        layer_specific_file_name = "spatio_temporal_voxel_layer.yaml"
        output_file_name = "nav2_params_generated_with_stvl.yaml"
    else:
        layer_specific_file_name = "voxel_layer.yaml"
        output_file_name = "nav2_params_generated_with_voxel.yaml"
        
    layer_specific_file_path = os.path.join(config_dir, layer_specific_file_name)
    
    # Output the generated file to a writable temporary location or a known build/log directory
    # For simplicity, placing it in the config_dir for now, but /tmp or a build artifact dir is better
    generated_params_output_path = os.path.join(config_dir, output_file_name)

    print(f"Generating Nav2 params: base='{base_nav2_params_file}', layer='{layer_specific_file_path}', output='{generated_params_output_path}'")
    merge_yaml_files(base_nav2_params_file, layer_specific_file_path, generated_params_output_path)
    nav2_params_file_to_use = generated_params_output_path

    
    # base_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(slam_package_path, 'launch/include/robot.launch.py')),
    #     launch_arguments={
    #         'sim': sim,
    #         'master_name': master_name,
    #         'robot_name': robot_name
    #     }.items(),
    # )
    
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation_package_path, 'launch/include/bringup.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': os.path.join(maps_path, map_name + '.yaml'),
            'params_file': nav2_params_file_to_use,
            'namespace': robot_name,
            'use_namespace': use_namespace,
            'autostart': 'true',
            'use_teb': use_teb,
        }.items(),
    )

    bringup_launch = GroupAction(
     actions=[
         PushRosNamespace(robot_name),
        #  base_launch,
         TimerAction(
             period=2.0,  # 延时等待其它节点启动好
             actions=[navigation_launch],
         ),
      ]
    )

    return [sim_arg, map_name_arg, master_name_arg, robot_name_arg, use_teb_arg, bringup_launch]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
