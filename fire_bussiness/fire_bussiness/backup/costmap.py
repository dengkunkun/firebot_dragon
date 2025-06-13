import rclpy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, Pose, PoseStamped, TransformStamped
import numpy as np
import math
from typing import List, Tuple, Optional
import tf2_ros
import tf2_geometry_msgs
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

class CostmapUtil:
    """
    灭火位置计算工具类，不继承Node
    需要外部提供logger、tf_buffer、tf_broadcaster和node实例用于订阅
    """
    def __init__(self, node: rclpy.node.Node, logger, tf_buffer: tf2_ros.Buffer, tf_broadcaster: tf2_ros.TransformBroadcaster):
        self.node = node  # 用于创建订阅和获取时钟
        self.logger = logger
        self.tf_buffer = tf_buffer
        self.tf_broadcaster = tf_broadcaster
        
        self.logger.info("=== CostmapUtil Tool Started ===")
        
        # Global Costmap的QoS配置
        costmap_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        
        # 使用外部node创建订阅
        self.costmap_subscriber = self.node.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self.global_costmap_callback, costmap_qos
        )
        self.logger.info("Subscribed to /global_costmap/costmap topic")

        self.local_costmap_subscriber = self.node.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self.local_costmap_callback, costmap_qos
        )
        self.logger.info("Subscribed to /local_costmap/costmap topic")

        # 地图数据
        self.global_map_data = None
        self.global_map_info = None
        self.local_map_data = None
        self.local_map_info = None

        # 参数配置
        self.min_distance = 0.5
        self.max_distance = 1.0
        self.robot_radius = 0.4
        self.safety_margin = 0.2
        self.free_threshold = 50
        self.occupied_threshold = 80

        self.logger.info(
            f"CostmapUtil parameters: min_dist={self.min_distance}m, max_dist={self.max_distance}m, "
            f"robot_radius={self.robot_radius}m, safety_margin={self.safety_margin}m"
        )

    def global_costmap_callback(self, msg: OccupancyGrid):
        """Global Costmap回调函数"""
        self.logger.debug(f"Received global costmap: {msg.info.width}x{msg.info.height}")
        self.global_map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.global_map_info = msg.info

    def local_costmap_callback(self, msg: OccupancyGrid):
        """Local Costmap回调函数"""
        self.logger.debug(f"Received local costmap: {msg.info.width}x{msg.info.height}")
        self.local_map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.local_map_info = msg.info

    def get_robot_position(self) -> Optional[Tuple[float, float]]:
        """获取机器人在map坐标系下的位置"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)
            )
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            return robot_x, robot_y
        except Exception as e:
            self.logger.warn(f"Failed to get robot position: {e}")
            return None

    def publish_extinguish_tf(self, pose_stamped: PoseStamped):
        """发布最优停靠点的TF"""
        t = TransformStamped()
        
        t.header.stamp = self.node.get_clock().now().to_msg()  # 使用外部node的时钟
        t.header.frame_id = "map"
        t.child_frame_id = "extinguish_position"
        
        t.transform.translation.x = pose_stamped.pose.position.x
        t.transform.translation.y = pose_stamped.pose.position.y
        t.transform.translation.z = pose_stamped.pose.position.z
        
        t.transform.rotation = pose_stamped.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)
        self.logger.info("Published extinguish_position TF")

    def get_best_approach_pose(self, target_x: float, target_y: float) -> Optional[PoseStamped]:
        """获取最佳停靠位置，返回PoseStamped，方向指向目标点"""
        positions = self.find_approach_positions(target_x, target_y)
        if not positions:
            self.logger.warn("No best position found")
            return None
            
        best_pos = positions[0]
        best_x, best_y = best_pos[0], best_pos[1]
        
        # 计算朝向
        yaw = math.atan2(target_y - best_y, target_x - best_x)
        
        # 构造PoseStamped
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.node.get_clock().now().to_msg()
        pose_stamped.header.frame_id = "map"
        
        pose_stamped.pose.position.x = best_x
        pose_stamped.pose.position.y = best_y
        pose_stamped.pose.position.z = 0.0
        
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = math.sin(yaw / 2.0)
        pose_stamped.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.logger.info(f"Best approach pose: ({best_x:.3f}, {best_y:.3f}), yaw: {math.degrees(yaw):.1f}°")
        
        # 发布TF
        self.publish_extinguish_tf(pose_stamped)
        
        return pose_stamped

    def is_ready(self) -> bool:
        """检查CostmapUtil是否准备就绪"""
        return self.global_map_data is not None

    def world_to_map(self, world_x: float, world_y: float, use_local: bool = False) -> Tuple[int, int]:
        """世界坐标转地图对应像素点坐标"""
        map_info = self.local_map_info if use_local else self.global_map_info
        
        if not map_info:
            self.logger.warn(f"{'Local' if use_local else 'Global'} map info not available for coordinate conversion")
            return None, None

        map_x = int((world_x - map_info.origin.position.x) / map_info.resolution)
        map_y = int((world_y - map_info.origin.position.y) / map_info.resolution)
        return map_x, map_y

    def map_to_world(self, map_x: int, map_y: int, use_local: bool = False) -> Tuple[float, float]:
        """地图像素点坐标转世界坐标"""
        map_info = self.local_map_info if use_local else self.global_map_info
        
        if not map_info:
            return None, None

        world_x = map_x * map_info.resolution + map_info.origin.position.x
        world_y = map_y * map_info.resolution + map_info.origin.position.y
        return world_x, world_y

    def is_valid_position(self, map_x: int, map_y: int, use_local: bool = False) -> bool:
        """检查位置是否未占用且在地图范围内"""
        map_data = self.local_map_data if use_local else self.global_map_data
        map_info = self.local_map_info if use_local else self.global_map_info
        
        if map_data is None or map_info is None:
            return False

        height, width = map_data.shape
        if map_x < 0 or map_x >= width or map_y < 0 or map_y >= height:
            return False

        cost_value = map_data[map_y, map_x]
        return cost_value >= 0 and cost_value < self.free_threshold

    def check_area_clearance(self, center_x: int, center_y: int, radius_cells: int, use_local: bool = False) -> bool:
        """检查指定半径内的区域是否清空"""
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy <= radius_cells * radius_cells:
                    if not self.is_valid_position(center_x + dx, center_y + dy, use_local):
                        return False
        return True

    def calculate_openness_score(self, center_x: int, center_y: int, check_radius: int, use_local: bool = False) -> float:
        """计算位置的空旷程度评分"""
        map_data = self.local_map_data if use_local else self.global_map_data
        
        if map_data is None:
            return 0.0

        free_cells = 0
        total_cells = 0
        cost_sum = 0

        for dy in range(-check_radius, check_radius + 1):
            for dx in range(-check_radius, check_radius + 1):
                dist = math.sqrt(dx * dx + dy * dy)
                if dist <= check_radius:
                    x, y = center_x + dx, center_y + dy
                    if (0 <= x < map_data.shape[1] and 0 <= y < map_data.shape[0]):
                        total_cells += 1
                        cost_value = map_data[y, x]
                        
                        if cost_value >= 0 and cost_value < self.free_threshold:
                            free_cells += 1
                            cost_sum += cost_value

        if total_cells == 0:
            return 0.0
            
        free_ratio = free_cells / total_cells
        avg_cost = cost_sum / max(free_cells, 1)
        cost_score = max(0, (self.free_threshold - avg_cost) / self.free_threshold)
        
        return free_ratio * 0.8 + cost_score * 0.2

    def validate_position_with_local_costmap(self, world_x: float, world_y: float) -> bool:
        """使用local costmap验证位置是否可用"""
        if not self.is_point_in_local_costmap(world_x, world_y):
            return True  # 不在local costmap范围内，跳过检查
            
        if self.local_map_data is None or self.local_map_info is None:
            self.logger.warn("Local costmap data not available for validation")
            return True  # local costmap不可用，跳过检查
            
        # 转换到local costmap坐标
        map_x, map_y = self.world_to_map(world_x, world_y, use_local=True)
        if map_x is None:
            return False
            
        # 检查机器人半径范围内的清空情况
        robot_radius_cells = int((self.robot_radius + self.safety_margin) / self.local_map_info.resolution)
        is_clear = self.check_area_clearance(map_x, map_y, robot_radius_cells, use_local=True)
        
        if not is_clear:
            self.logger.info(f"Position ({world_x:.3f}, {world_y:.3f}) rejected by local costmap validation")
            
        return is_clear

    def find_approach_positions(self, target_x: float, target_y: float) -> List[Tuple[float, float, float]]:
        """找到目标点附近的最佳停靠位置"""
        self.logger.info(f"Searching approach positions for target: ({target_x:.3f}, {target_y:.3f})")

        if self.global_map_data is None or self.global_map_info is None:
            self.logger.warn("Global costmap data not available")
            return []

        # 检查是否需要local costmap验证
        use_local_validation = self.is_point_in_local_costmap(target_x, target_y)
        if use_local_validation:
            self.logger.info("Target is within local costmap range - will use local costmap for validation")
        else:
            self.logger.info("Target is outside local costmap range - using global costmap only")

        target_map_x, target_map_y = self.world_to_map(target_x, target_y, use_local=False)
        if target_map_x is None:
            self.logger.error("Failed to convert target coordinates")
            return []

        min_radius_cells = int(self.min_distance / self.global_map_info.resolution)
        max_radius_cells = int(self.max_distance / self.global_map_info.resolution)
        robot_radius_cells = int((self.robot_radius + self.safety_margin) / self.global_map_info.resolution)
        openness_check_radius = int(1.5 / self.global_map_info.resolution)

        candidates = []
        checked_positions = 0
        valid_positions = 0
        local_rejected = 0

        for dy in range(-max_radius_cells, max_radius_cells + 1):
            for dx in range(-max_radius_cells, max_radius_cells + 1):
                dist_cells = math.sqrt(dx * dx + dy * dy)

                if min_radius_cells <= dist_cells <= max_radius_cells:
                    checked_positions += 1
                    candidate_x = target_map_x + dx
                    candidate_y = target_map_y + dy

                    # 首先用global costmap检查
                    if self.check_area_clearance(candidate_x, candidate_y, robot_radius_cells, use_local=False):
                        # 转换到世界坐标
                        world_x, world_y = self.map_to_world(candidate_x, candidate_y, use_local=False)
                        
                        # 如果在local costmap范围内，进行额外验证
                        if use_local_validation:
                            if not self.validate_position_with_local_costmap(world_x, world_y):
                                local_rejected += 1
                                continue
                        
                        valid_positions += 1

                        # 计算评分（使用global costmap）
                        openness_score = self.calculate_openness_score(
                            candidate_x, candidate_y, openness_check_radius, use_local=False
                        )

                        ideal_dist = (self.min_distance + self.max_distance) / 2
                        actual_dist = dist_cells * self.global_map_info.resolution
                        distance_score = 1.0 - abs(actual_dist - ideal_dist) / ideal_dist

                        total_score = openness_score * 0.7 + distance_score * 0.3
                        candidates.append((world_x, world_y, total_score))

        self.logger.info(
            f"Checked {checked_positions} positions, "
            f"passed global: {valid_positions + local_rejected}, "
            f"rejected by local: {local_rejected}, "
            f"final candidates: {len(candidates)}"
        )

        if not candidates:
            self.logger.warn("No valid candidates found!")
            return []

        candidates.sort(key=lambda x: x[2], reverse=True)
        self.logger.info(f"Position:{candidates[0][0]:.3f} {candidates[0][1]:.3f} Best candidate score: {candidates[0][2]:.3f}")

        # 过滤相近位置
        filtered_candidates = []
        min_separation = 0.3

        for candidate in candidates:
            is_isolated = True
            for existing in filtered_candidates:
                dist = math.sqrt((candidate[0] - existing[0]) ** 2 + (candidate[1] - existing[1]) ** 2)
                if dist < min_separation:
                    is_isolated = False
                    break

            if is_isolated:
                filtered_candidates.append(candidate)

            if len(filtered_candidates) >= 10:
                break

        self.logger.info(f"Found {len(filtered_candidates)} isolated approach positions")
        return filtered_candidates

    def analyze_and_get_pose(self, target_x: float, target_y: float) -> Optional[PoseStamped]:
        """分析目标点并返回最优停靠姿态"""
        self.logger.info("=== Starting Analysis ===")
        self.logger.info(f"Fire target coordinates: ({target_x:.3f}, {target_y:.3f})")

        # 获取机器人位置信息
        robot_pos = self.get_robot_position()
        if robot_pos:
            robot_x, robot_y = robot_pos
            distance_to_target = math.sqrt((target_x - robot_x)**2 + (target_y - robot_y)**2)
            self.logger.info(f"Robot position: ({robot_x:.3f}, {robot_y:.3f}), distance to target: {distance_to_target:.3f}m")

        # 获取最佳位置
        best_pose = self.get_best_approach_pose(target_x, target_y)
        if best_pose:
            self.logger.info(
                f"Analysis complete. Best extinguish pose: "
                f"pos=({best_pose.pose.position.x:.3f}, {best_pose.pose.position.y:.3f}), "
                f"yaw={math.degrees(math.atan2(2*(best_pose.pose.orientation.w*best_pose.pose.orientation.z), 1-2*best_pose.pose.orientation.z**2)):.1f}°"
            )
        else:
            self.logger.warn("Analysis complete. No valid approach pose found")

        return best_pose

    def is_ready(self) -> bool:
        """检查CostmapUtil是否准备就绪"""
        return self.global_map_data is not None

    def wait_for_costmap(self, timeout_sec: float = 10.0) -> bool:
        """等待costmap数据可用"""
        import time
        start_time = time.time()
        
        while self.global_map_data is None:
            if time.time() - start_time > timeout_sec:
                self.logger.error("Timeout waiting for costmap data")
                return False
            time.sleep(0.1)
            
        self.logger.info("Costmap data ready")
        return True
    def is_point_in_local_costmap(self, target_x: float, target_y: float) -> bool:
        """检查目标点是否在local costmap范围内"""
        if self.local_map_info is None:
            return False
            
        # 计算目标点在local costmap中的像素坐标
        map_x = int((target_x - self.local_map_info.origin.position.x) / self.local_map_info.resolution)
        map_y = int((target_y - self.local_map_info.origin.position.y) / self.local_map_info.resolution)
        
        # 检查是否在范围内
        if (0 <= map_x < self.local_map_info.width and 0 <= map_y < self.local_map_info.height):
            self.logger.debug(f"Target ({target_x:.3f}, {target_y:.3f}) is within local costmap")
            return True
        else:
            self.logger.debug(f"Target ({target_x:.3f}, {target_y:.3f}) is outside local costmap")
            return False

# def main(args=None):
#     rclpy.init(args=args)

#     costmap_util = CostmapUtil()

#     def test_pose_generation():
#         costmap_util.logger.info("Test thread started - waiting for costmap data...")
#         import time

#         timeout_count = 0
#         while costmap_util.global_map_data is None:
#             rclpy.spin_once(costmap_util, timeout_sec=0.1)
#             time.sleep(0.1)
#             timeout_count += 1

#             if timeout_count % 50 == 0:
#                 costmap_util.logger.info(f"Still waiting for global costmap data... ({timeout_count/10:.1f}s)")

#             if timeout_count > 300:
#                 costmap_util.logger.error("Timeout waiting for costmap data!")
#                 return

#         costmap_util.logger.info("Global costmap data received! Starting analysis...")

#         # 测试火源目标点
#         target_x, target_y = -0.57759857, 0.72457
#         best_pose = costmap_util.analyze_and_get_pose(target_x, target_y)
        
#         if best_pose:
#             costmap_util.logger.info("SUCCESS: Generated extinguish pose and TF")
#         else:
#             costmap_util.logger.error("FAILED: Could not generate extinguish pose")

#         costmap_util.logger.info("Test completed")

#     try:
#         costmap_util.logger.info("Starting main execution...")
#         import threading

#         test_thread = threading.Thread(target=test_pose_generation)
#         test_thread.daemon = True
#         test_thread.start()

#         costmap_util.logger.info("Main spin loop started")
#         rclpy.spin(costmap_util)
#     except KeyboardInterrupt:
#         costmap_util.logger.info("Received keyboard interrupt")
#     finally:
#         costmap_util.logger.info("Shutting down...")
#         costmap_util.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()