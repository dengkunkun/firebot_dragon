这里用于存放幻儿机器人移植的方案
motion_jetson 底盘控制的正解和逆解
    订阅：set_motor
    发布：odom_raw
odom_filter_jetson 将imu和轮上编码器的数据融合成odom
    订阅：imu_corrected;odom_raw
    发布：odom
scan_filter_jetson 雷达过滤
    订阅：scan_raw
    发布：scan
slam_toolbox_jetson 

rtabmap_jetson

nav2_jetson

bringup_jetson 所有硬件启动


usb_link:
    /imu_raw frame_id: imu_link

motion_jetson:
    /odom_raw:
        base_frame_id: base_footprint
        odom_frame_id: odom_raw