#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

class BoxMarkerPublisher(Node):
    def __init__(self):
        super().__init__('box_marker_pub')
        # 发布到 /marker 话题
        self.pub = self.create_publisher(Marker, 'marker', 10)
        self.timer = self.create_timer(0.5, self.publish_marker)

        # Marker 的公共属性
        self.marker = Marker()
        self.marker.header.frame_id = 'map'         # 坐标系
        self.marker.ns = 'demo'                    # 命名空间
        self.marker.id = 0                         # 唯一 ID
        self.marker.type = Marker.CUBE             # 立方体
        self.marker.action = Marker.ADD            # 添加（或 MODIFY）
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.0
        self.marker.pose.orientation.w = 1.0       # 无旋转
        # 立方体尺寸
        self.marker.scale.x = 1.0
        self.marker.scale.y = 2.0
        self.marker.scale.z = 1.0
        # 颜色 RGBA
        self.marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
        self.get_logger().info('BoxMarkerPublisher started')

    def publish_marker(self):
        # 更新时间戳
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.marker)

def main(args=None):
    rclpy.init(args=args)
    node = BoxMarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()