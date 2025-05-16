from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # Get the package share directory
    pkg_dir = get_package_share_directory('bringup_jetson')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Load config file
    ekf_param = os.path.join(pkg_dir, 'config', 'ekf.yaml')

    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_param, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/tf', 'tf'),  #订阅
            ('/tf_static', 'tf_static'), #订阅
            ('odometry/filtered', 'odom'), #发布
            ('cmd_vel', 'controller/cmd_vel') 
        ],
    )
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[
            {'fixed_frame': "imu_link",
            'use_mag': False,
            'publish_tf': False,
            'world_frame': "enu",
            'orientation_stddev': 0.05}
        ],
        remappings=[
            ('/tf', 'tf'),
            # ('/imu/data_raw', '/handsfree/imu'),  # 替换为实际 IMU 话题
            ('/imu/data_raw', '/imu_raw'),
            ('imu/data', 'imu')
            ]
    )

    return LaunchDescription([
        imu_filter_node,
        ekf_filter_node,
    ])
    
if __name__ == '__main__':
    # 创建一个LaunchDescription对象
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()