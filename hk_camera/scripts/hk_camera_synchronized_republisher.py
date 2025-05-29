import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import copy # For deepcopying messages

class SynchronizedRepublisher(Node):
    def __init__(self):
        super().__init__('synchronized_republisher')
        
        self.latest_camera_info = None
        self.camera_info_received = False

        # Publishers for the synchronized topics
        self.image_pub = self.create_publisher(Image, '/synced/image_raw', 10)
        self.cam_info_pub = self.create_publisher(CameraInfo, '/synced/camera_info', 10)

        # Subscriber for CameraInfo
        self.cam_info_sub = self.create_subscription(
            CameraInfo,
            '/depth_cam/color/camera_info',
            self.camera_info_callback,
            10) # QoS profile depth

        # Subscriber for Image
        self.image_sub = self.create_subscription(
            Image,
            '/depth_cam/color/image_raw',
            self.image_callback,
            10) # QoS profile depth
        
        self.get_logger().info('Synchronized republisher node started.')
        self.get_logger().info('Subscribing to /depth_cam/color/image_raw and /depth_cam/color/camera_info')
        self.get_logger().info('Publishing to /synced/image_raw and /synced/camera_info')

    def camera_info_callback(self, msg):
        self.latest_camera_info = msg
        if not self.camera_info_received:
            self.camera_info_received = True
            self.get_logger().info('First CameraInfo message received.')

    def image_callback(self, image_msg):
        if not self.camera_info_received or self.latest_camera_info is None:
            self.get_logger().warn('CameraInfo not yet received. Skipping image message.')
            return

        current_stamp = image_msg.header.stamp

        # Publish the image message as is (it has the desired timestamp)
        self.image_pub.publish(image_msg)

        # Create a deepcopy of the latest camera_info to modify its timestamp
        cam_info_to_publish = copy.deepcopy(self.latest_camera_info)
        cam_info_to_publish.header.stamp = current_stamp
        # Ensure the frame_id of cam_info_to_publish is consistent.
        # It will inherit frame_id from self.latest_camera_info.
        # image_msg.header.frame_id should ideally match cam_info_to_publish.header.frame_id.
        
        self.cam_info_pub.publish(cam_info_to_publish)
        
        self.get_logger().info(f'Republished image and camera_info with timestamp: {current_stamp.sec}.{current_stamp.nanosec}')

def main(args=None):
    rclpy.init(args=args)
    node = SynchronizedRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()