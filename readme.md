通过环境变量配置不同硬件和公共参数

#暂时不做，后期实际有了再改
export need_compile=True
export LIDAR_TYPE=G4
export MACHINE_TYPE=JetRover_Tank
export MASTER=/
export HOST=/

参数：
轮距：	0.614
直径：	0.2
use_sim_time	
最大速度
最大加速度


请求灭火action：
    目的地坐标
    超时时间
feedback：
    当前状态：无法计划；导航失败；导航成功；开始扫描；扫描不到；扫描成功；开始灭火；灭火成功；灭火失败
    当前位置
    火焰距离
result：
    灭火成功/灭火失败
    msg
