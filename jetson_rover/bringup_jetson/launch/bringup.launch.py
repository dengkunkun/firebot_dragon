from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource # MODIFIED: Corrected import path


def generate_launch_description():
    # Get package directories
    diff_drive_robot_dir= get_package_share_directory('diff_drive_robot')
    # imu_pkg_dir = get_package_share_directory('imu_hfi_a9')
    ydlidar_pkg_dir = get_package_share_directory('ydlidar_g4')
    astra_pro_plus_pkg_dir = get_package_share_directory('orbbec_camera')
    dabai_dcw_pkg_dir = get_package_share_directory('dabai_dcw')
    bringup_pkg_dir = get_package_share_directory('bringup_jetson')
    motion_pkg_dir = get_package_share_directory('motion_jetson')
    print(f'astra_pro_plus_pkg_dir: {astra_pro_plus_pkg_dir}')
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    lidar_frame = LaunchConfiguration('lidar_frame', default='laser_frame')
    scan_raw = LaunchConfiguration('scan_raw', default='scan_raw')
    camera_name = LaunchConfiguration('camera_name', default='astra_pro_plus')
    
    diff_robot_desc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(diff_drive_robot_dir, 'launch', 'rsp.launch.py')
        )
    )
    
    # Include IMU launch
    # imu_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(imu_pkg_dir, 'launch', 'imu.launch.py')
    #     )
    # )
    motion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(motion_pkg_dir, 'launch', 'motion.launch.py')
        )
    )
    
    # Include YDLidar launch
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ydlidar_pkg_dir, 'launch', 'ydlidar_g4.launch.py')
        ),
        launch_arguments={
            'lidar_frame': lidar_frame,
            'scan_raw': scan_raw
        }.items()
    )
    # if camera_name == 'astra_pro_plus':
    depth_camera = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(astra_pro_plus_pkg_dir, 'launch', 'astra_pro_plus.launch.xml')
        ),
    )
    # else:
    #     depth_camera = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             os.path.join(dabai_dcw_pkg_dir, 'launch', 'dabai_dcw.launch.py')
    #         ),
    #     )
    # Include odom filter launch
    odom_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg_dir, 'launch', 'odom_filter.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    lidar_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg_dir, 'launch', 'lidar_filter.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Create USB link node
    usblink_node = Node(
        package='usblink',
        executable='usblink',
        name='usblink',
        output='screen',
        emulate_tty=True
    )
    
    # Return launch description
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='false',
                             description='Use simulation time if true'),
        DeclareLaunchArgument('lidar_frame', default_value='laser_frame',
                             description='Frame ID for lidar'),
        DeclareLaunchArgument('scan_raw', default_value='scan_raw',
                             description='Topic name for raw scan data'),
        
        # Launch files and nodes
        diff_robot_desc_launch,
        usblink_node,
        # imu_launch,
        motion_launch,
        lidar_launch,
        depth_camera,
        lidar_filter_launch,
        odom_filter_launch,
    ])