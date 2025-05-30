#!/usr/bin/env python3
# filepath: /home/cat/firebot_dragon/src/hk_camera/scripts/ros2_rmw_benchmark.py

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import argparse
import time
import threading
import statistics
from std_msgs.msg import String
from std_msgs.msg import UInt8MultiArray  # 改用UInt8MultiArray
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped
import psutil
import os
import struct

class BenchmarkPublisher(Node):
    def __init__(self, topic_name, message_size, qos_profile, publish_rate):
        super().__init__('benchmark_publisher')
        
        self.topic_name = topic_name
        self.message_size = message_size
        self.publish_rate = publish_rate
        
        # 创建发布者 - 改用UInt8MultiArray
        self.publisher = self.create_publisher(UInt8MultiArray, topic_name, qos_profile)
        
        # 生成测试数据
        self.test_data = self._generate_test_data(message_size)
        
        # 统计变量
        self.messages_sent = 0
        self.bytes_sent = 0
        self.start_time = None
        
        # 创建定时器
        timer_period = 1.0 / publish_rate  # 发布频率
        self.timer = self.create_timer(timer_period, self.publish_message)
        
        self.get_logger().info(f'发布者启动: 话题={topic_name}, 消息大小={message_size}字节, 频率={publish_rate}Hz')
        
    def _generate_test_data(self, size):
        """生成指定大小的测试数据"""
        # 预留8字节用于时间戳
        data_size = max(8, size)
        return [i % 256 for i in range(data_size)]
    
    def publish_message(self):
        if self.start_time is None:
            self.start_time = time.time()
            
        # 添加时间戳到数据中
        current_time = time.time_ns()
        
        # 创建消息
        msg = UInt8MultiArray()
        
        # 将时间戳编码为8字节，然后填充剩余数据
        timestamp_data = list(struct.pack('<Q', current_time))  # 8字节时间戳
        
        # 确保消息大小正确
        if len(self.test_data) >= 8:
            msg.data = timestamp_data + self.test_data[8:]
        else:
            msg.data = timestamp_data
            
        # 确保消息大小与预期一致
        if len(msg.data) < self.message_size:
            padding = [0] * (self.message_size - len(msg.data))
            msg.data.extend(padding)
        elif len(msg.data) > self.message_size:
            msg.data = msg.data[:self.message_size]
        
        self.publisher.publish(msg)
        
        self.messages_sent += 1
        self.bytes_sent += len(msg.data)
        
        # 每秒统计一次
        if self.messages_sent % self.publish_rate == 0:
            elapsed_time = time.time() - self.start_time
            throughput = self.bytes_sent / elapsed_time / 1024 / 1024  # MB/s
            msg_rate = self.messages_sent / elapsed_time  # msg/s
            
            self.get_logger().info(
                f'发布统计: {self.messages_sent}条消息, '
                f'{throughput:.2f} MB/s, {msg_rate:.1f} msg/s'
            )

class BenchmarkSubscriber(Node):
    def __init__(self, topic_name, qos_profile):
        super().__init__('benchmark_subscriber')
        
        self.topic_name = topic_name
        
        # 创建订阅者 - 改用UInt8MultiArray
        self.subscription = self.create_subscription(
            UInt8MultiArray,
            topic_name,
            self.message_callback,
            qos_profile
        )
        
        # 统计变量
        self.messages_received = 0
        self.bytes_received = 0
        self.latencies = []
        self.start_time = None
        self.last_stat_time = time.time()
        self.last_msg_count = 0
        self.last_byte_count = 0
        
        # 性能监控
        self.cpu_usage = []
        self.memory_usage = []
        
        # 统计定时器
        self.stat_timer = self.create_timer(1.0, self.print_statistics)
        
        self.get_logger().info(f'订阅者启动: 话题={topic_name}')
        
    def message_callback(self, msg):
        if self.start_time is None:
            self.start_time = time.time()
            
        receive_time = time.time_ns()
        
        # 解析时间戳
        if len(msg.data) >= 8:
            try:
                # 从前8字节解析时间戳
                timestamp_bytes = bytes(msg.data[:8])
                send_time = struct.unpack('<Q', timestamp_bytes)[0]
                latency = (receive_time - send_time) / 1e6  # 转换为毫秒
                self.latencies.append(latency)
                
                # 只保留最近1000个延迟数据
                if len(self.latencies) > 1000:
                    self.latencies.pop(0)
            except (struct.error, ValueError) as e:
                # 时间戳解析失败，跳过延迟计算
                pass
        
        self.messages_received += 1
        self.bytes_received += len(msg.data)
        
    def print_statistics(self):
        current_time = time.time()
        
        if self.messages_received == 0:
            return
            
        # 计算速率
        time_delta = current_time - self.last_stat_time
        msg_delta = self.messages_received - self.last_msg_count
        byte_delta = self.bytes_received - self.last_byte_count
        
        msg_rate = msg_delta / time_delta if time_delta > 0 else 0
        throughput = byte_delta / time_delta / 1024 / 1024 if time_delta > 0 else 0  # MB/s
        
        # 计算延迟统计
        latency_stats = ""
        if self.latencies:
            avg_latency = statistics.mean(self.latencies)
            min_latency = min(self.latencies)
            max_latency = max(self.latencies)
            latency_stats = f', 延迟: 平均{avg_latency:.2f}ms, 最小{min_latency:.2f}ms, 最大{max_latency:.2f}ms'
        
        # 系统资源使用
        cpu_percent = psutil.cpu_percent()
        memory_percent = psutil.virtual_memory().percent
        
        self.get_logger().info(
            f'接收统计: {self.messages_received}条消息, '
            f'{throughput:.2f} MB/s, {msg_rate:.1f} msg/s'
            f'{latency_stats}, CPU: {cpu_percent:.1f}%, 内存: {memory_percent:.1f}%'
        )
        
        # 更新统计基准
        self.last_stat_time = current_time
        self.last_msg_count = self.messages_received
        self.last_byte_count = self.bytes_received

# 其余函数保持不变...
def create_qos_profile(reliability, history, depth, durability):
    """创建QoS配置"""
    qos = QoSProfile(depth=depth)
    
    if reliability == 'reliable':
        qos.reliability = ReliabilityPolicy.RELIABLE
    else:
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        
    if history == 'keep_last':
        qos.history = HistoryPolicy.KEEP_LAST
    else:
        qos.history = HistoryPolicy.KEEP_ALL
        
    if durability == 'transient_local':
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    else:
        qos.durability = DurabilityPolicy.VOLATILE
        
    return qos

def parse_size(size_str):
    """解析大小字符串，支持K, M, G后缀"""
    size_str = size_str.upper().strip()
    
    if size_str.endswith('K'):
        return int(float(size_str[:-1]) * 1024)
    elif size_str.endswith('M'):
        return int(float(size_str[:-1]) * 1024 * 1024)
    elif size_str.endswith('G'):
        return int(float(size_str[:-1]) * 1024 * 1024 * 1024)
    else:
        return int(size_str)

def main():
    parser = argparse.ArgumentParser(description='ROS2 RMW带宽和延迟基准测试工具')
    parser.add_argument('mode', choices=['pub', 'sub', 'both'], 
                       help='运行模式: pub(发布者), sub(订阅者), both(同时运行)')
    parser.add_argument('--topic', default='/benchmark_topic', 
                       help='测试话题名称 (默认: /benchmark_topic)')
    parser.add_argument('--size', default='1M', 
                       help='单个消息大小，支持K/M/G后缀 (默认: 1M)')
    parser.add_argument('--rate', type=int, default=10, 
                       help='发布频率 Hz (默认: 10)')
    parser.add_argument('--reliability', choices=['reliable', 'best_effort'], 
                       default='reliable', help='QoS可靠性 (默认: reliable)')
    parser.add_argument('--history', choices=['keep_last', 'keep_all'], 
                       default='keep_last', help='QoS历史策略 (默认: keep_last)')
    parser.add_argument('--depth', type=int, default=10, 
                       help='QoS队列深度 (默认: 10)')
    parser.add_argument('--durability', choices=['volatile', 'transient_local'], 
                       default='volatile', help='QoS持久性 (默认: volatile)')
    parser.add_argument('--duration', type=int, default=60, 
                       help='测试持续时间(秒) (默认: 60)')
    parser.add_argument('--rmw', help='指定RMW实现 (例如: rmw_cyclonedx_cpp)')
    
    args = parser.parse_args()
    
    # 设置RMW实现
    if args.rmw:
        os.environ['RMW_IMPLEMENTATION'] = args.rmw
        print(f"使用RMW实现: {args.rmw}")
    
    # 解析消息大小
    message_size = parse_size(args.size)
    
    # 创建QoS配置
    qos_profile = create_qos_profile(
        args.reliability, args.history, args.depth, args.durability
    )
    
    print(f"测试配置:")
    print(f"  话题: {args.topic}")
    print(f"  消息大小: {message_size:,} 字节 ({args.size})")
    print(f"  发布频率: {args.rate} Hz")
    print(f"  QoS: {args.reliability}, {args.history}, 深度={args.depth}, {args.durability}")
    print(f"  测试时长: {args.duration} 秒")
    print(f"  当前RMW: {os.environ.get('RMW_IMPLEMENTATION', '默认')}")
    print("-" * 60)
    
    rclpy.init()
    
    nodes = []
    
    try:
        if args.mode in ['pub', 'both']:
            pub_node = BenchmarkPublisher(args.topic, message_size, qos_profile, args.rate)
            nodes.append(pub_node)
            
        if args.mode in ['sub', 'both']:
            sub_node = BenchmarkSubscriber(args.topic, qos_profile)
            nodes.append(sub_node)
        
        if args.mode == 'both':
            # 并行运行发布者和订阅者
            executor = rclpy.executors.MultiThreadedExecutor()
            for node in nodes:
                executor.add_node(node)
                
            print("开始测试...")
            start_time = time.time()
            
            try:
                while rclpy.ok() and (time.time() - start_time) < args.duration:
                    executor.spin_once(timeout_sec=0.1)
            except KeyboardInterrupt:
                print("\n用户中断测试")
                
        else:
            # 单独运行发布者或订阅者
            node = nodes[0]
            print("开始测试...")
            start_time = time.time()
            
            try:
                while rclpy.ok() and (time.time() - start_time) < args.duration:
                    rclpy.spin_once(node, timeout_sec=0.1)
            except KeyboardInterrupt:
                print("\n用户中断测试")
                
        print("\n测试完成!")
        
        # 打印最终统计
        if args.mode in ['sub', 'both'] and len(nodes) > 0:
            sub_node = nodes[-1] if args.mode == 'both' else nodes[0]
            if hasattr(sub_node, 'messages_received') and sub_node.messages_received > 0:
                total_time = time.time() - sub_node.start_time
                avg_throughput = sub_node.bytes_received / total_time / 1024 / 1024
                avg_msg_rate = sub_node.messages_received / total_time
                
                print(f"\n最终统计:")
                print(f"  总接收消息: {sub_node.messages_received:,}")
                print(f"  总接收数据: {sub_node.bytes_received / 1024 / 1024:.2f} MB")
                print(f"  平均吞吐量: {avg_throughput:.2f} MB/s")
                print(f"  平均消息率: {avg_msg_rate:.1f} msg/s")
                
                if sub_node.latencies:
                    print(f"  延迟统计:")
                    print(f"    平均: {statistics.mean(sub_node.latencies):.2f} ms")
                    print(f"    最小: {min(sub_node.latencies):.2f} ms")
                    print(f"    最大: {max(sub_node.latencies):.2f} ms")
                    if len(sub_node.latencies) > 1:
                        print(f"    标准差: {statistics.stdev(sub_node.latencies):.2f} ms")
                    
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()