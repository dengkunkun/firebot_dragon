#!/usr/bin/env python3
"""
HK Camera Frame Rate Monitor
订阅 /camera/image_color 话题并计算帧率
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import time
import signal
import sys

class CompressedFrameRateMonitor(Node):
    def __init__(self):
        super().__init__('compressed_frame_rate_monitor')
        
        # 优化的QoS配置
        qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )
        
        # 创建订阅者
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_color/compressed',
            self.image_callback,
            qos_profile)
        
        # 统计变量
        self.frame_count = 0
        self.total_frames = 0
        self.total_bytes = 0
        self.start_time = time.time()
        self.last_frame_time = None
        self.last_report_time = time.time()
        self.instantaneous_fps = 0.0
        self.average_fps = 0.0
        
        # 图像信息
        self.image_width = 0
        self.image_height = 0
        self.image_format = ""
        
        # 性能监控
        self.processing_times = []
        self.max_processing_time = 0.0
        self.compression_ratios = []
        
        self.get_logger().info("压缩图像帧率监控启动")

    def image_callback(self, msg):
        callback_start = time.time()
        current_time = time.time()
        
        # 检测消息延迟
        msg_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        current_ros_time = self.get_clock().now().nanoseconds / 1e9
        message_latency = current_ros_time - msg_timestamp
        
        # 更新帧计数
        self.frame_count += 1
        self.total_frames += 1
        
        # 计算瞬时帧率
        if self.total_frames > 1:
            time_diff = current_time - self.last_frame_time
            if time_diff > 0:
                self.instantaneous_fps = 1.0 / time_diff
                
                if time_diff > 1.0:
                    self.get_logger().warn(f"检测到帧率异常: {time_diff:.2f}秒间隔")
        
        self.last_frame_time = current_time
        
        # 解压缩图像获取尺寸信息（偶尔执行）
        if self.total_frames == 1 or self.total_frames % 100 == 0:
            try:
                # 解码压缩图像
                np_arr = np.frombuffer(msg.data, np.uint8)
                if msg.format == "jpeg":
                    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                else:  # png
                    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
                
                if cv_image is not None:
                    self.image_height, self.image_width = cv_image.shape[:2]
                    self.image_format = msg.format
                    
                    # 计算压缩比
                    uncompressed_size = cv_image.shape[0] * cv_image.shape[1] * cv_image.shape[2] if len(cv_image.shape) == 3 else cv_image.shape[0] * cv_image.shape[1]
                    compression_ratio = uncompressed_size / len(msg.data)
                    self.compression_ratios.append(compression_ratio)
                    
                    if len(self.compression_ratios) > 100:
                        self.compression_ratios.pop(0)
                        
            except Exception as e:
                self.get_logger().warn(f"解码图像失败: {e}")
        
        # 累计数据量
        self.total_bytes += len(msg.data)
        
        # 计算处理时间
        callback_end = time.time()
        processing_time = (callback_end - callback_start) * 1000
        self.processing_times.append(processing_time)
        
        if len(self.processing_times) > 100:
            self.processing_times.pop(0)
        
        self.max_processing_time = max(self.max_processing_time, processing_time)
        
        # 每5秒报告一次
        if current_time - self.last_report_time >= 5.0:
            self.report_statistics(current_time, message_latency)
            self.frame_count = 0
            self.last_report_time = current_time

    def report_statistics(self, current_time, latency):
        total_elapsed = current_time - self.start_time
        if total_elapsed > 0:
            self.average_fps = self.total_frames / total_elapsed
        
        period_elapsed = current_time - self.last_report_time
        period_fps = self.frame_count / period_elapsed if period_elapsed > 0 else 0.0
        
        # 计算数据速率 (Mbps)
        data_rate_mbps = (self.total_bytes * 8) / (total_elapsed * 1024 * 1024) if total_elapsed > 0 else 0
        
        # 处理时间统计
        avg_processing_time = sum(self.processing_times) / len(self.processing_times) if self.processing_times else 0
        
        # 压缩比统计
        avg_compression_ratio = sum(self.compression_ratios) / len(self.compression_ratios) if self.compression_ratios else 0
        
        print("\n" + "="*80)
        print(f"压缩图像接收统计")
        print(f"运行时间: {total_elapsed:.1f} 秒")
        print(f"总帧数: {self.total_frames}")
        print(f"瞬时帧率: {self.instantaneous_fps:.2f} FPS")
        print(f"短期帧率(5秒): {period_fps:.2f} FPS")
        print(f"平均帧率: {self.average_fps:.2f} FPS")
        print(f"图像尺寸: {self.image_width}x{self.image_height}")
        print(f"图像格式: {self.image_format}")
        print(f"数据速率: {data_rate_mbps:.2f} Mbps")
        print(f"总数据量: {self.total_bytes:,} 字节")
        print(f"平均压缩比: {avg_compression_ratio:.1f}:1")
        print(f"消息延迟: {latency:.3f} 秒")
        print(f"处理时间: 平均{avg_processing_time:.2f}ms, 最大{self.max_processing_time:.2f}ms")
        
        # 健康检查
        if period_fps < 5.0 and self.total_frames > 10:
            print("⚠️  警告: 帧率过低")
        elif abs(period_fps - self.average_fps) > 10.0:
            print("⚠️  警告: 帧率波动较大")
        else:
            print("✅ 帧率正常")
        
        print("="*80)

def signal_handler(sig, frame):
    print("\n正在停止监控...")
    rclpy.shutdown()
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)
    
    rclpy.init()
    monitor = CompressedFrameRateMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()