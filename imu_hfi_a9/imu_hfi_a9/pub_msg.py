#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import serial
import struct
import math
import serial.tools.list_ports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
# from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler

'''
aa55 14 23e04800 ac9a1e83839a1c3f6b100341568d29c1 170e 
 
包头：aa55 
包装长度：14 
有效负载信息：23e04800 
有效负载内容：ac9a1e83    839a1c3f    6b100341    568d29c1  
CRC 校验：170e 
 
前面的 ac 9a 1e 83  表示时间戳 
 
需 要 调 整 顺 序 变 成  0x831e9aac（ 十 进 制 为  2199820972）， 表示 十进制 为
2199.820972秒，不是IEEE-754浮点数

aa55 2c 29ec4100 4029706b9d66383a508faab9da33c0b9a6e74d3c0267b9bb03048
0bf25bfac3d4fd40fbdcd4c4a3f 5dfb 
 
包头：aa55 
包装长度：2c 
有效负载信息：29ec4100 
有效负载内容：4029706b 9d66383a 508faab9 da33c0b9 a6e74d3c 0267b9bb 
030480bf 25bfac3d 4fd40fbd cd4c4a3f 
CRC 校验：5dfb
数据分别是：陀螺仪、加速计、磁力计
'''

def find_ttyUSB():
    print('imu 默认串口为 /dev/ttyUSB0, 若识别多个串口设备, 请在 launch 文件中修改 imu 对应的串口')
    posts = [port.device for port in serial.tools.list_ports.comports() if 'USB' in port.device]
    print('当前电脑所连接的 {} 串口设备共 {} 个: {}'.format('USB', len(posts), posts))

def checkSum(list_data, check_data):
    # data = bytearray(list_data)
    data=list_data
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for i in range(8):
            if (crc & 1) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return hex(((crc & 0xff) << 8) + (crc >> 8)) == hex(check_data[0] << 8 | check_data[1])

def parse_imu_data(raw_data):
    # print(f"buff_data: {[hex(x)[2:].zfill(2) for x in raw_data]}")
    raw_data=list(raw_data)
    ieee_data = []
    raw_data.reverse()
    # print(f"raw_data: {[hex(x)[2:].zfill(2) for x in raw_data]}")
    for i in range(0, len(raw_data)-4, 4):
        data2str = hex(raw_data[i] | 0xff00)[4:6] + hex(raw_data[i + 1] | 0xff00)[4:6] + hex(raw_data[i + 2] | 0xff00)[4:6] + hex(raw_data[i + 3] | 0xff00)[4:6]
        ieee_data.append(struct.unpack('>f', bytes.fromhex(data2str))[0])
        
    timestamp = (raw_data[-4] << 24) | (raw_data[-3] << 16) | (raw_data[-2] << 8) | raw_data[-1]
    ieee_data.append(float(timestamp)/1000000.0)
    ieee_data.reverse()
    return ieee_data

class HFIa9IMUNode(Node):
    def __init__(self):
        super().__init__('hfi_a9_imu')
        find_ttyUSB()
        self.port = self.declare_parameter('port', '/dev/ttyUSB0').value
        self.baudrate = self.declare_parameter('baudrate', 921600).value
        self.gra_normalization = self.declare_parameter('gra_normalization', True).value
        self.cali_static_bias= self.declare_parameter('cali_static_bias', False).value  #减去初始静态的误差
        self.cali_angular_noise= self.declare_parameter('cali_angular_noise', False).value  #减去角速度的噪声
        self.declare_parameter('angular_velocity_threshold', 0.005)  # Add threshold parameter
        self.angular_velocity_threshold = self.get_parameter('angular_velocity_threshold').value

        
        self.imu_pub = self.create_publisher(Imu, 'handsfree/imu', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'handsfree/mag', 10)

        self.imu_msg = Imu()
        self.mag_msg = MagneticField()
        self.imu_msg.orientation_covariance = [0.0001, 0, 0,
                                    0, 0.0001, 0,
                                    0, 0, 0.0001]
        self.imu_msg.angular_velocity_covariance = [0.0001, 0, 0,
                                        0, 0.0001, 0,
                                        0, 0, 0.0001]
        self.imu_msg.linear_acceleration_covariance = [0.0001, 0, 0,
                                            0, 0.0001, 0,
                                            0, 0, 0.0001]
        
        self.angularVelocity = [0, 0, 0]
        self.acceleration = [0, 0, 0]
        self.magnetometer = [0, 0, 0]
        self.angle_degree = [0, 0, 0]
        
        self.serial_head=b'\xaa\x55'
        self.remain_data=b''
        
        # 添加静态误差补偿相关变量
        self.static_acc_x = []  # 存储1秒内的x轴加速度数据
        self.static_acc_y = []
        self.static_acc_z = []
        self.static_acc_bias = [0.0, 0.0, 0.0]  # 存储计算出的静态误差
        self.static_calibration_samples = 300  # 1秒内的采样点数
        self.is_calibrated = False  # 是否已完成校准
        # 添加原始重力加速度标准值
        self.gravity_standard = 9.8  # 标准重力加速度


        try:
            self.hf_imu = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=0.5)
            if self.hf_imu.isOpen():
                self.get_logger().info("串口打开成功...")
            else:
                self.hf_imu.open()
                self.get_logger().info("打开串口成功...")
        except Exception as e:
            self.get_logger().error(f"串口打开失败: {e}")
            rclpy.shutdown()  # 优雅地关闭ROS2
            return

        self.timer = self.create_timer(1.0/300, self.read_serial)

    def handleSerialData(self, raw_data):
        now = self.get_clock().now().to_msg()
        self.imu_msg.header.stamp = now
        self.imu_msg.header.frame_id = "base_link"
        self.mag_msg.header.stamp = now
        self.mag_msg.header.frame_id = "base_link"

        angle_radian = [self.angle_degree[i] * math.pi / 180 for i in range(3)]
        qua = quaternion_from_euler(angle_radian[0], -angle_radian[1], -angle_radian[2])

        self.imu_msg.orientation.x = qua[0]
        self.imu_msg.orientation.y = qua[1]
        self.imu_msg.orientation.z = qua[2]
        self.imu_msg.orientation.w = qua[3]
        
        if self.cali_angular_noise :
            filtered_angular_velocity = [
                v if abs(v) >= self.angular_velocity_threshold else 0.0 
                for v in self.angularVelocity
            ]
            self.imu_msg.angular_velocity.x = filtered_angular_velocity[0]
            self.imu_msg.angular_velocity.y = filtered_angular_velocity[1] 
            self.imu_msg.angular_velocity.z = filtered_angular_velocity[2]
        else:
            self.imu_msg.angular_velocity.x = float(self.angularVelocity[0])
            self.imu_msg.angular_velocity.y = float(self.angularVelocity[1])
            self.imu_msg.angular_velocity.z = float(self.angularVelocity[2])

        # 如果启用静态误差校准且还未校准完成
        if self.cali_static_bias and not self.is_calibrated:
            # 收集原始加速度数据
            self.static_acc_x.append(self.acceleration[0])
            self.static_acc_y.append(self.acceleration[1])
            self.static_acc_z.append(self.acceleration[2])
            
            if len(self.static_acc_x) >= self.static_calibration_samples:
                # 计算平均值
                avg_x = sum(self.static_acc_x) / len(self.static_acc_x)
                avg_y = sum(self.static_acc_y) / len(self.static_acc_y)
                avg_z = sum(self.static_acc_z) / len(self.static_acc_z)
                
                # 计算重力向量的模
                gravity_magnitude = math.sqrt(avg_x**2 + avg_y**2 + avg_z**2)
                
                # 计算期望值 - 理想情况下，静止时应该只有z轴有-1g的重力加速度
                expected_x = 0.0
                expected_y = 0.0
                expected_z = -1.0  # 期望z轴为-1g
                
                # 计算偏差
                self.static_acc_bias[0] = avg_x - expected_x
                self.static_acc_bias[1] = avg_y - expected_y
                self.static_acc_bias[2] = avg_z - expected_z
                
                self.is_calibrated = True
                self.get_logger().info(
                    f"Static calibration completed:\n"
                    f"Original average: x={avg_x:.3f}, y={avg_y:.3f}, z={avg_z:.3f}\n"
                    f"Gravity magnitude: {gravity_magnitude:.3f}\n"
                    f"Computed bias: x={self.static_acc_bias[0]:.3f}, "
                    f"y={self.static_acc_bias[1]:.3f}, z={self.static_acc_bias[2]:.3f}"
                )
                
                # 清空校准数据
                self.static_acc_x.clear()
                self.static_acc_y.clear()
                self.static_acc_z.clear()

        # 应用校准后的数据处理
        acc_x = float(self.acceleration[0])
        acc_y = float(self.acceleration[1])
        acc_z = float(self.acceleration[2])
        
        if self.cali_static_bias and self.is_calibrated:
            # 减去静态偏差
            acc_x -= self.static_acc_bias[0]
            acc_y -= self.static_acc_bias[1]
            acc_z -= self.static_acc_bias[2]
        
        # 计算归一化系数
        acc_k = math.sqrt(acc_x**2 + acc_y**2 + acc_z**2)
        if acc_k < 0.1:
            acc_k = 1.0

        # 发布消息
        if self.gra_normalization:
            # 应用标准重力加速度
            self.imu_msg.linear_acceleration.x = float(acc_x * self.gravity_standard)
            self.imu_msg.linear_acceleration.y = float(acc_y * self.gravity_standard)
            self.imu_msg.linear_acceleration.z = float(acc_z * self.gravity_standard)
        else:
            self.imu_msg.linear_acceleration.x = float(acc_x)
            self.imu_msg.linear_acceleration.y = float(acc_y)
            self.imu_msg.linear_acceleration.z = float(acc_z)

        self.mag_msg.magnetic_field.x = float(self.magnetometer[0])
        self.mag_msg.magnetic_field.y = float(self.magnetometer[1])
        self.mag_msg.magnetic_field.z = float(self.magnetometer[2])

        self.imu_pub.publish(self.imu_msg)
        self.mag_pub.publish(self.mag_msg)

    def read_serial(self):
        while True:
            try:
                buff_count = self.hf_imu.inWaiting()
            except Exception as e:
                self.get_logger().error(f"exception: {e}")
                self.get_logger().error("imu 失去连接，接触不良，或断线")
                self.hf_imu.close()  # 关闭串口
                rclpy.shutdown()  # 优雅地关闭ROS2
                return
            else:
                if buff_count > 0x14+5:
                    buff_data = self.hf_imu.read(buff_count)
                    if self.remain_data:
                        buff_count = buff_count + len(self.remain_data)
                        buff_data = self.remain_data + buff_data
                        self.remain_data = b''
                    for i in range(0, buff_count):
                        if buff_count -i <0x14+5:
                            self.remain_data = buff_data[i:]
                            return
                        if buff_data[i]==0xaa and buff_data[i+1]==0x55 and (buff_data[i+2]==0x14 or buff_data[i+2]==0x2c):
                            # print(f"buff_data: {[hex(x)[2:].zfill(2) for x in buff_data[i:i+buff_count]]}")
                            if buff_data[i+2]==0x14 and buff_count-i < 0x14+5:
                                self.get_logger().warn("imu 数据包长度异常: 0x14")
                                continue
                            elif buff_data[i+2]==0x2c and buff_count-i < 0x2c+5:
                                self.get_logger().warn("imu 数据包长度异常: 0x2c")
                                continue
                            if checkSum(buff_data[i+2:i+3+buff_data[i+2]], buff_data[i+3+buff_data[i+2]:i+5+buff_data[i+2]]):
                                parsed_data= parse_imu_data(buff_data[i+7:i+3+buff_data[i+2]])
                                if buff_data[i+2]==0x14:
                                    angle_degree_timestamp = parsed_data[0]
                                    self.angle_degree = parsed_data[1:4]
                                    self.get_logger().debug(f"角度数据: {self.angle_degree}")
                                    self.get_logger().debug(f"时间戳: {angle_degree_timestamp}")
                                    self.handleSerialData(buff_data[i])
                                elif buff_data[i+2]==0x2c:
                                    origin_time= parsed_data[0]
                                    self.angularVelocity = parsed_data[1:4]
                                    self.acceleration = parsed_data[4:7]
                                    self.magnetometer = parsed_data[7:10]
                                    self.get_logger().debug(f"角速度: {self.angularVelocity}")
                                    self.get_logger().debug(f"加速度: {self.acceleration}")
                                    self.get_logger().debug(f"磁力计: {self.magnetometer}")
                                    self.get_logger().debug(f"时间戳: {origin_time}")
                                    self.handleSerialData(buff_data[i])
                                i += buff_data[i+2]+5

def main(args=None):
    rclpy.init(args=args)
    node = HFIa9IMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()