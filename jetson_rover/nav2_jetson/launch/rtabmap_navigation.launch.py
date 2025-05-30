import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import PushRosNamespace
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction, TimerAction

def launch_setup(context):
    slam_package_path = get_package_share_directory('slam_toolbox_jetson')
    navigation_package_path = get_package_share_directory('nav2_jetson')

    sim = LaunchConfiguration('sim', default='false').perform(context)
    map_name = LaunchConfiguration('map', default='').perform(context)
    robot_name = LaunchConfiguration('robot_name', default='/').perform(context)
    master_name = LaunchConfiguration('master_name', default='/').perform(context)

    sim_arg = DeclareLaunchArgument('sim', default_value=sim)
    map_name_arg = DeclareLaunchArgument('map', default_value=map_name)
    master_name_arg = DeclareLaunchArgument('master_name', default_value=master_name)
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value=robot_name)

    use_sim_time = 'true' if sim == 'true' else 'false'
    use_namespace = 'true' if robot_name != '/' else 'false'
    frame_prefix = '' if robot_name == '/' else '%s/'%robot_name
    topic_prefix = '' if robot_name == '/' else '/%s'%robot_name
    map_frame = '{}map'.format(frame_prefix)
    odom_frame = '{}odom'.format(frame_prefix)
    base_frame = '{}base_footprint'.format(frame_prefix)
    depth_camera_topic = '{}/depth_cam/depth/image_raw'.format(topic_prefix)
    depth_camera_info = '{}/depth_cam/color/camera_info'.format(topic_prefix)
    rgb_camera_topic = '{}/depth_cam/color/image_raw'.format(topic_prefix)
    odom_topic = '{}/odom'.format(topic_prefix)
    scan_topic = '{}/scan'.format(topic_prefix)

    
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation_package_path, 'launch/include/bringup.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': os.path.join(slam_package_path, 'maps', map_name + '.yaml'),
            'params_file': os.path.join(navigation_package_path, 'config', 'nav2_params.yaml'),
            'namespace': robot_name,
            'use_namespace': use_namespace,
            'autostart': 'true',
            'rtabmap': 'true',
        }.items(),
    )

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_package_path, 'launch/include/rtabmap.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    bringup_launch = GroupAction(
     actions=[
         PushRosNamespace(robot_name),
         TimerAction(
             period=5.0,  # 延时等待其它节点启动好
             actions=[navigation_launch],
         ),
         rtabmap_launch
      ]
    )

    return [sim_arg, master_name_arg, robot_name_arg, bringup_launch]

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
