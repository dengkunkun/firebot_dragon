#! /usr/bin/env python3
from geometry_msgs.msg import (
    PoseStamped,
    PoseWithCovarianceStamped,
    Twist,
    Point32,
    Polygon,
)
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav2_simple_commander.costmap_2d import PyCostmap2D
from nav2_simple_commander.footprint_collision_checker import FootprintCollisionChecker
import rclpy, tf2_ros
import tf2_geometry_msgs
import traceback, copy

from rclpy.duration import Duration
# from builtin_interfaces.msg import Duration
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from nav2_msgs.action import Spin
from nav2_msgs.action import (
    NavigateToPose,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter_client import AsyncParameterClient
from rclpy.parameter import Parameter
from fire_interfaces.srv import SetInitialPoseSrv, SetNavigationGoalSrv
from fire_interfaces.action import RotateSearch, FireExtinguishing
import math
from tf_transformations import euler_from_quaternion
import time
from .log import ColorLogger
from .socket_shell import TCPSocketServer
import numpy as np

from fire_interfaces.msg import FireInfo
from action_msgs.msg import GoalStatus

import inspect
from typing import get_type_hints


def auto_convert_args(func, str_args: list):
    """
    功能说明：根据函数签名自动转换字符串参数为对应类型

    参数说明：
        func: 目标函数对象
        str_args (list): 字符串参数列表

    返回值说明：
        tuple: 转换后的参数元组

    实现说明：
        1. 获取函数签名和类型注解
        2. 根据类型注解转换每个参数
        3. 支持常见类型的自动转换
    """
    try:
        # 获取函数签名
        sig = inspect.signature(func)
        type_hints = get_type_hints(func)

        converted_args = []
        param_names = list(sig.parameters.keys())

        # 跳过self参数
        start_idx = 1 if param_names and param_names[0] == "self" else 0

        for i, str_arg in enumerate(str_args):
            param_idx = start_idx + i
            if param_idx >= len(param_names):
                # 如果参数超出定义，直接使用字符串
                converted_args.append(str_arg)
                continue

            param_name = param_names[param_idx]
            param_type = type_hints.get(param_name, str)

            # 根据类型进行转换
            if param_type == float:
                converted_args.append(float(str_arg))
            elif param_type == int:
                converted_args.append(int(str_arg))
            elif param_type == bool:
                converted_args.append(str_arg.lower() in ("true", "1", "yes", "on"))
            elif param_type == str:
                converted_args.append(str_arg)
            else:
                # 对于其他类型，尝试直接构造
                try:
                    converted_args.append(param_type(str_arg))
                except:
                    converted_args.append(str_arg)

        return tuple(converted_args)

    except Exception as e:
        # 转换失败时返回原始参数
        return tuple(str_args)


def create_auto_wrapper(method_name: str):
    """
    功能说明：为指定方法创建自动类型转换的包装函数

    参数说明：
        method_name (str): 目标方法名称

    返回值说明：
        callable: 包装后的函数

    实现说明：
        1. 动态创建包装函数
        2. 自动进行参数类型转换
        3. 调用原方法并返回结果
    """

    def wrapper(self, *str_args) -> str:
        try:
            # 获取原方法
            original_method = getattr(self, method_name)

            # 自动转换参数
            converted_args = auto_convert_args(original_method, str_args)

            # 调用原方法
            result = original_method(*converted_args)

            # 格式化返回结果
            if isinstance(result, tuple):
                if len(result) == 2 and hasattr(result[0], "pose"):
                    # 处理 (PoseStamped, score) 类型的返回值
                    pose, score = result
                    if pose:
                        return f"SUCCESS: Best pose at ({pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}) score:{score:.3f}"
                    else:
                        return f"FAILED: No valid pose found, score:{score:.3f}"
                else:
                    return f"SUCCESS: {result}"
            else:
                return f"SUCCESS: {result}"

        except Exception as e:
            return (
                f"ERROR in {method_name}: {str(e)}\nTraceback: {traceback.format_exc()}"
            )

    return wrapper


def auto_socket_command(original_method_name: str = None):
    """
    功能说明：装饰器，自动为方法创建socket命令版本

    参数说明：
        original_method_name (str): 原方法名，如果不指定则根据装饰的方法名推断

    返回值说明：
        callable: 装饰器函数

    实现说明：
        1. 自动推断或指定原方法名
        2. 创建socket命令版本的方法
        3. 自动进行参数类型转换
    """

    def decorator(cmd_method):
        def wrapper(self, *str_args) -> str:
            # 推断原方法名
            if original_method_name:
                target_method_name = original_method_name
            else:
                # 从命令方法名推断原方法名 (去掉_cmd后缀)
                target_method_name = cmd_method.__name__.replace("_cmd", "")

            try:
                # 获取原方法
                original_method = getattr(self, target_method_name)

                # 自动转换参数
                converted_args = auto_convert_args(original_method, str_args)

                # 调用原方法
                result = original_method(*converted_args)

                # 格式化返回结果
                return format_method_result(result, target_method_name)

            except Exception as e:
                return f"ERROR in {target_method_name}: {str(e)}\nTraceback: {traceback.format_exc()}"

        return wrapper

    return decorator


def format_method_result(result, method_name: str) -> str:
    """
    功能说明：格式化方法返回结果为字符串

    参数说明：
        result: 方法返回值
        method_name (str): 方法名称

    返回值说明：
        str: 格式化后的结果字符串

    实现说明：
        1. 根据返回值类型进行不同的格式化
        2. 特殊处理PoseStamped等ROS消息类型
        3. 提供统一的错误信息格式
    """
    if isinstance(result, tuple) and len(result) == 2:
        pose, score = result
        if hasattr(pose, "pose") and pose is not None:
            return f"SUCCESS: Best pose at ({pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}) score:{score:.3f}"
        else:
            return f"FAILED: No valid pose found, score:{score:.3f}"
    elif result is None:
        return f"FAILED: {method_name} returned None"
    elif isinstance(result, bool):
        return f"{'SUCCESS' if result else 'FAILED'}: {method_name} completed"
    else:
        return f"SUCCESS: {result}"


class NavigationNode(Node):
    from enum import Enum

    class FireDetectState(Enum):
        FireDetected = 1
        NoFireDetected = 2
        FireDetectionError = 3
        FireDetectStop = 4

    class CostmapAdapter:
        """适配器类，包装 nav2_msgs/Costmap 使其兼容 PyCostmap2D"""

        def __init__(self, nav2_costmap):
            # 复制所有原始属性
            self.header = nav2_costmap.header
            self.data = nav2_costmap.data
            self.metadata = nav2_costmap.metadata

            # 创建兼容的 info 属性
            from nav_msgs.msg import MapMetaData

            self.info = MapMetaData()
            self.info.map_load_time = nav2_costmap.metadata.map_load_time
            self.info.resolution = nav2_costmap.metadata.resolution
            self.info.width = nav2_costmap.metadata.size_x
            self.info.height = nav2_costmap.metadata.size_y
            self.info.origin = nav2_costmap.metadata.origin

    def __init__(self):
        super().__init__("navigation_node")
        self.logger = ColorLogger(self)
        self.logger.info("Initializing NavigationNode...")
        self.navigator = BasicNavigator()
        self.goal_active = False
        self._action_goal_handle = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.excluded_directions = {}  # 存储被排除的方向 {fire_key: set(angle_ranges)}
        self.failed_positions = (
            {}
        )  # 存储失败的位置 {fire_key: [(x, y, timestamp), ...]}
        self.direction_exclusion_timeout = 60.0  # 排除方向的超时时间（秒）

        self.scan_active = False

        # 使用官方的碰撞检测器
        self.collision_checker = FootprintCollisionChecker()
        self.global_costmap = None
        self.local_costmap = None

        # 机器人足迹 - 可以根据实际尺寸调整
        self.robot_footprint = self._create_robot_footprint()
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.tcp_server_host = "0.0.0.0"
        self.tcp_server_port = 12345
        self.tcp_socket_server = TCPSocketServer(
            host=self.tcp_server_host,
            port=self.tcp_server_port,
            command_handler_callback=self._handle_socket_command,
            logger=self.get_logger(),
            rclpy_ok_check=rclpy.ok,
        )
        if not self.tcp_socket_server.start():
            self.logger.error(
                "Failed to start TCP socket server during node initialization."
            )

        self.fire_info_sub = self.create_subscription(
            FireInfo, "/hk_camera/fire_info", self.fire_info_callback, 10
        )
        self.fire_info = None

        self.nav2_init_start_time = time.time()
        self.nav2_init_timer = self.create_timer(1, self.check_nav2_init_timeout)
        self.nav2_initialized = False
        
        self.print_info_timer = self.create_timer(1,self.print_info)

        self.motion_params_client = AsyncParameterClient(self, "/motion_jetson")

        self.nav_cb_group = ReentrantCallbackGroup()
        self.fire_extinguishing_action_server = ActionServer(
            self,
            FireExtinguishing,
            "FireExtinguishing",
            execute_callback=self.execute_fire_extinguishing_callback,
            goal_callback=self.goal_fire_extinguishing_callback,
            handle_accepted_callback=self.handle_accepted_fire_extinguishing_callback,
            cancel_callback=self.cancel_fire_extinguishing_callback,
            callback_group=self.nav_cb_group,
        )
        self.fire_extinguishing_start_time = time.time()

        self.nav_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose", callback_group=self.nav_cb_group
        )

        self.spin_client = ActionClient(
            self, Spin, "spin", callback_group=self.nav_cb_group
        )
        self.logger.info("RotateSearch action server started.")

        self.logger.info("Navigation node started. Waiting for Nav2 to be active...")
        self.navigator.waitUntilNav2Active(localizer="robot_localization")

        # 获取costmap数据
        self._update_costmaps()

        self.nav2_initialized = True
        self.nav2_init_timer.cancel()
        self.logger.info("Nav2 is active.")
        
    def print_info(self):
        """打印当前状态信息"""
        robot_pose = self.get_robot_pose_in_map()
        if robot_pose:
            self.logger.info(
                f"Robot pose in map: x={robot_pose.pose.position.x:.3f}, y={robot_pose.pose.position.y:.3f}"
            )
        else:
            self.logger.warn("Failed to get robot pose in map.")

        if self.fire_info:
            self.logger.info(
                f"Fire detected at: x={self.fire_info.fire_position.x:.3f}, y={self.fire_info.fire_position.y:.3f}"
            )
        else:
            self.logger.info("No fire detected.")

    def _pub_cmd_vel(self, speed: float, turn: float):
        """发布速度指令"""
        cmd_vel = Twist()
        cmd_vel.linear.x = speed
        cmd_vel.angular.z = turn
        self.cmd_vel_pub.publish(cmd_vel)
        self.logger.debug(f"Published cmd_vel: linear={speed}, angular={turn}")

    def _create_robot_footprint(self, radius=0.4):
        """创建机器人足迹 - 圆形足迹"""
        footprint = Polygon()
        num_points = 16
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            point = Point32()
            point.x = radius * math.cos(angle)
            point.y = radius * math.sin(angle)
            point.z = 0.0
            footprint.points.append(point)
        return footprint

    def _update_costmaps(self):
        """更新全局和局部costmap"""
        try:
            # 获取全局costmap
            global_costmap_msg = self.navigator.getGlobalCostmap()
            self.logger.info(f"global_costmap_msg type: {type(global_costmap_msg)}")

            if global_costmap_msg:
                # 为 costmap 添加兼容的 info 属性
                compatible_costmap = self._make_costmap_compatible(global_costmap_msg)

                self.global_costmap = PyCostmap2D(compatible_costmap)
                self.collision_checker.setCostmap(self.global_costmap)
                self.logger.info("Global costmap updated")

            # 获取局部costmap
            local_costmap_msg = self.navigator.getLocalCostmap()
            if local_costmap_msg:
                # 为 costmap 添加兼容的 info 属性
                compatible_local_costmap = self._make_costmap_compatible(
                    local_costmap_msg
                )

                self.local_costmap = PyCostmap2D(compatible_local_costmap)
                self.logger.info("Local costmap updated")

        except Exception as e:
            self.logger.error(f"Failed to update costmaps: {e}")
            self.logger.error(f"Full traceback:\n{traceback.format_exc()}")

    def _make_costmap_compatible(self, costmap_msg):
        """使 nav2_msgs/Costmap 兼容 PyCostmap2D"""
        # 使用适配器类包装原始消息
        compatible_costmap = self.CostmapAdapter(costmap_msg)

        self.logger.debug(
            f"Created compatible costmap: {compatible_costmap.info.width}x{compatible_costmap.info.height}, resolution={compatible_costmap.info.resolution}"
        )

        return compatible_costmap

    def _create_pose_stamped(self, x: float, y: float, yaw: float) -> PoseStamped:
        """创建PoseStamped消息"""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        # 四元数转换
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        return pose

    def _publish_extinguish_tf(self, pose_stamped: PoseStamped):
        """发布灭火位置的TF"""
        from geometry_msgs.msg import TransformStamped

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "extinguish_position"

        t.transform.translation.x = pose_stamped.pose.position.x
        t.transform.translation.y = pose_stamped.pose.position.y
        t.transform.translation.z = pose_stamped.pose.position.z
        t.transform.rotation = pose_stamped.pose.orientation

        self.tf_broadcaster.sendTransform(t)
        self.logger.info("Published extinguish_position TF")

    def calculate_footprint_average_cost(
        self, x: float, y: float, yaw: float, footprint: Polygon, sample_step: int = 2
    ) -> float:
        """
        计算机器人足迹范围内的平均cost值

        Args:
            x, y: 世界坐标位置
            yaw: 机器人朝向角度
            footprint: 机器人足迹多边形
            sample_step: 采样步长，每隔几个cell采样一次，默认为2

        Returns:
            平均cost值，如果无法计算则返回255（致命障碍物）
        """
        if not self.global_costmap:
            return 255.0

        try:
            # 将足迹转换到世界坐标系
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)

            # 计算足迹在世界坐标系中的边界
            world_points = []
            for point in footprint.points:
                # 旋转并平移到世界坐标
                world_x = x + (point.x * cos_yaw - point.y * sin_yaw)
                world_y = y + (point.x * sin_yaw + point.y * cos_yaw)
                world_points.append((world_x, world_y))

            # 计算边界框
            min_x = min(p[0] for p in world_points)
            max_x = max(p[0] for p in world_points)
            min_y = min(p[1] for p in world_points)
            max_y = max(p[1] for p in world_points)

            # 转换到地图坐标
            min_map_coords = self.global_costmap.worldToMapValidated(min_x, min_y)
            max_map_coords = self.global_costmap.worldToMapValidated(max_x, max_y)

            if min_map_coords[0] is None or max_map_coords[0] is None:
                return 255.0  # 超出地图范围

            min_mx, min_my = min_map_coords
            max_mx, max_my = max_map_coords

            # 确保边界在地图范围内
            min_mx = max(0, min_mx)
            min_my = max(0, min_my)
            max_mx = min(self.global_costmap.getSizeInCellsX() - 1, max_mx)
            max_my = min(self.global_costmap.getSizeInCellsY() - 1, max_my)

            # 采样计算平均cost
            total_cost = 0.0
            sample_count = 0
            lethal_count = 0  # 致命障碍物点数

            # 按步长采样
            for my in range(min_my, max_my + 1, sample_step):
                for mx in range(min_mx, max_mx + 1, sample_step):
                    # 转换回世界坐标检查是否在足迹内
                    world_coords = self.global_costmap.mapToWorld(mx, my)
                    sample_x, sample_y = world_coords

                    # 检查点是否在足迹多边形内
                    if self._point_in_polygon(sample_x, sample_y, world_points):
                        cost = self.global_costmap.getCostXY(mx, my)
                        total_cost += cost
                        sample_count += 1

                        # 统计致命障碍物
                        if cost >= 253:  # LETHAL_OBSTACLE
                            lethal_count += 1

            if sample_count == 0:
                return 255.0  # 没有有效采样点

            average_cost = total_cost / sample_count

            # 如果有致命障碍物，增加惩罚
            lethal_ratio = lethal_count / sample_count
            if lethal_ratio > 0.1:  # 超过10%的点是致命障碍物
                penalty = lethal_ratio * 100  # 根据致命障碍物比例增加惩罚
                average_cost = min(255.0, average_cost + penalty)

            self.logger.debug(
                f"Footprint average cost: {average_cost:.2f}, samples: {sample_count}, lethal_ratio: {lethal_ratio:.2f}"
            )

            return average_cost

        except Exception as e:
            self.logger.error(f"Error calculating footprint average cost: {e}")
            return 255.0

    def _point_in_polygon(self, x: float, y: float, polygon_points: list) -> bool:
        """
        使用射线法判断点是否在多边形内

        Args:
            x, y: 待检查的点坐标
            polygon_points: 多边形顶点列表 [(x1,y1), (x2,y2), ...]

        Returns:
            True如果点在多边形内，False否则

        从目标点向任意方向（通常是水平右方向）发射一条射线，统计这条射线与多边形边界的交点数量：
            奇数个交点 → 点在多边形内部
            偶数个交点 → 点在多边形外部
        """
        n = len(polygon_points)
        inside = False

        j = n - 1
        for i in range(n):
            xi, yi = polygon_points[i]
            xj, yj = polygon_points[j]

            if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
                inside = not inside
            j = i

        return inside

    def calculate_position_score(
        self,
        x: float,
        y: float,
        yaw: float,
        fire_x: float,
        fire_y: float,
        min_distance: float,
        max_distance: float,
    ) -> float:
        """
        综合评估位置的优劣程度

        Args:
            x, y, yaw: 候选位置和朝向
            fire_x, fire_y: 火源位置
            min_distance, max_distance: 距离范围

        Returns:
            综合评分 (0-1)，越高越好
        """
        try:
            # 1. 计算平均cost评分 (0-1，越高越好)
            avg_cost = self.calculate_footprint_average_cost(
                x, y, yaw, self.robot_footprint, sample_step=2
            )
            if avg_cost >= 253:  # 致命障碍物
                return 0.0

            # 将cost值转换为评分 (cost越低，评分越高)
            cost_score = max(0.0, (253 - avg_cost) / 253.0)

            # 2. 距离评分 (0-1，越接近理想距离越高)
            actual_distance = math.sqrt((x - fire_x) ** 2 + (y - fire_y) ** 2)
            ideal_distance = (min_distance + max_distance) / 2.0
            distance_error = abs(actual_distance - ideal_distance) / ideal_distance
            distance_score = max(0.0, 1.0 - distance_error)

            # 3. 旋转对齐评分 (检查朝向是否正确指向火源)
            expected_yaw = math.atan2(fire_y - y, fire_x - x)
            yaw_error = abs(self._normalize_angle(yaw - expected_yaw))
            orientation_score = max(0.0, 1.0 - yaw_error / math.pi)

            # 4. 开放性评分 (检查周围的开放程度)
            openness_score = self._calculate_openness_score(x, y, radius=1.0)

            # 综合评分 (可调整权重)
            total_score = (
                cost_score * 0.4  # 通行性权重40%
                + distance_score * 0.15  # 距离权重25%
                + orientation_score * 0.15  # 朝向权重15%
                + openness_score * 0.30  # 开放性权重20%
            )

            self.logger.debug(
                f"Position ({x:.3f}, {y:.3f}) scores: cost={cost_score:.3f}, "
                f"distance={distance_score:.3f}, orient={orientation_score:.3f}, "
                f"openness={openness_score:.3f}, total={total_score:.3f}"
            )

            return total_score

        except Exception as e:
            self.logger.error(f"Error calculating position score: {e}")
            return 0.0

    def _normalize_angle(self, angle: float) -> float:
        """将角度归一化到 [-π, π] 范围"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _calculate_openness_score(
        self, x: float, y: float, radius: float = 1.0
    ) -> float:
        """
        计算位置周围的开放程度

        Args:
            x, y: 世界坐标位置
            radius: 检查半径 (米)

        Returns:
            开放性评分 (0-1)，越高表示越开放
        """
        if not self.global_costmap:
            return 0.0

        try:
            resolution = self.global_costmap.getResolution()
            radius_cells = int(radius / resolution)

            center_coords = self.global_costmap.worldToMapValidated(x, y)
            if center_coords[0] is None:
                return 0.0

            center_mx, center_my = center_coords

            free_cells = 0
            total_cells = 0

            # 在圆形区域内采样 (每隔2个cell采样一次)
            for dy in range(-radius_cells, radius_cells + 1, 2):
                for dx in range(-radius_cells, radius_cells + 1, 2):
                    if dx * dx + dy * dy <= radius_cells * radius_cells:
                        mx = center_mx + dx
                        my = center_my + dy

                        if (
                            0 <= mx < self.global_costmap.getSizeInCellsX()
                            and 0 <= my < self.global_costmap.getSizeInCellsY()
                        ):

                            cost = self.global_costmap.getCostXY(mx, my)
                            total_cells += 1

                            if cost < 100:  # 认为是自由空间
                                free_cells += 1

            if total_cells == 0:
                return 0.0

            openness_score = free_cells / total_cells
            return openness_score

        except Exception as e:
            self.logger.error(f"Error calculating openness score: {e}")
            return 0.0

    def fire_info_callback(self, msg: FireInfo):
        self.fire_info = msg

    def is_fire_detected(self,timeout_s: float = 1.0):
        return self.fire_info and time.time() - self.fire_info.header.stamp.sec < timeout_s

    def get_fire_position(self):
        if not self.fire_info:
            return None
        return self.fire_info.fire_position.x, self.fire_info.fire_position.y
    
    def get_robot_diff_from_fire_in_map(self):
        '''
        计算当前机器人位置与火源的map坐标中的x，y方向距离和yaw
        '''
        if not self.fire_info:
            return None, None, None
        fire_x = self.fire_info.fire_position_map.x
        fire_y = self.fire_info.fire_position_map.y
        robot_pose = self.get_robot_pose_in_map()
        if robot_pose is None:
            return None, None, None
        robot_x = robot_pose.pose.position.x
        robot_y = robot_pose.pose.position.y
        # 计算x，y方向距离
        dx = fire_x - robot_x
        dy = fire_y - robot_y
        # 计算yaw角度
        from tf_transformations import euler_from_quaternion
        _, _, robot_yaw = euler_from_quaternion(
            [
                robot_pose.pose.orientation.x,
                robot_pose.pose.orientation.y,
                robot_pose.pose.orientation.z,
                robot_pose.pose.orientation.w,
            ]
        )
        # 计算火源的yaw角度
        fire_yaw = math.atan2(dy, dx)
        # 计算yaw差值
        yaw_diff = self._normalize_angle(fire_yaw - robot_yaw)
        return dx, dy, yaw_diff

    def is_same_fire_point(self, fire_x: float, fire_y: float):
        """
        检查当前火源位置是否与给定位置相同
        """
        if not self.fire_info:
            return False
        if (
            abs(self.fire_info.fire_position.x - fire_x) < 0.1
            and abs(self.fire_info.fire_position.y - fire_y) < 0.1
        ):
            return True
        return False

    def check_nav2_init_timeout(self):
        """Check if Nav2 initialization has timed out"""
        return

    def motion_params_angular_max_z(self, value=None):
        if value is not None:
            self.logger.info(f"Setting angular_max_z to {value}")
            future = self.motion_params_client.set_parameters(
                [Parameter("angular_max_z", Parameter.Type.DOUBLE, value)]
            )
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            if result:
                self.logger.info("Parameter set successfully.")
            else:
                self.logger.error("Failed to set parameter.")
        else:
            future = self.motion_params_client.get_parameters(["angular_max_z"])
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            if result:
                return result.values[0].double_value
            else:
                self.logger.error("Failed to get parameter.")
                return None
    def  motion_params_stop(self, value=None):
        if value is not None:
            self.logger.info(f"Setting stop to {value}")
            future = self.motion_params_client.set_parameters(
                [Parameter("stop", Parameter.Type.BOOL, value)]
            )
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            if result:
                self.logger.info("Parameter set successfully.")
            else:
                self.logger.error("Failed to set parameter.")
        else:
            future = self.motion_params_client.get_parameters(["stop"])
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            if result:
                return result.values[0].bool_value
            else:
                self.logger.error("Failed to get parameter.")
                return None
    def get_robot_pose_in_map(self):
        """
        获取机器人在map坐标系中的完整姿态（位置+朝向）

        Returns:
            geometry_msgs.msg.PoseStamped: 完整的机器人姿态，失败时返回None
        """
        try:
            # 查询base_link在map中的变换
            transform = self.tf_buffer.lookup_transform(
                "base_link",
                "map",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )

            # 构造PoseStamped
            robot_pose = PoseStamped()
            robot_pose.header.stamp = self.get_clock().now().to_msg()
            robot_pose.header.frame_id = "map"

            # 位置
            robot_pose.pose.position.x = transform.transform.translation.x
            robot_pose.pose.position.y = transform.transform.translation.y
            robot_pose.pose.position.z = transform.transform.translation.z

            # 朝向
            robot_pose.pose.orientation = transform.transform.rotation

            # 计算yaw角度用于日志
            from tf_transformations import euler_from_quaternion

            orientation_list = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ]
            (_, _, yaw) = euler_from_quaternion(orientation_list)

            self.logger.debug(
                f"Robot pose in map: pos=({robot_pose.pose.position.x:.3f}, {robot_pose.pose.position.y:.3f}), yaw={math.degrees(yaw):.1f}°"
            )

            return robot_pose

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            self.logger.error(f"Failed to get robot pose stamped in map: {e}")
            return None
        except Exception as e:
            self.logger.error(f"Unexpected error getting robot pose stamped: {e}")
            return None

    def is_robot_moved(self, last_pose: PoseStamped,diff_distance:float,diff_yaw:float) -> bool:
        """
        检查机器人是否移动
        """
        current_pose = self.get_robot_pose_in_map()
        if current_pose is None or last_pose is None:
            return False

        # 计算位置差异
        dx = current_pose.pose.position.x - last_pose.pose.position.x
        dy = current_pose.pose.position.y - last_pose.pose.position.y
        distance = math.sqrt(dx * dx + dy * dy)

        # 计算朝向差异
        from tf_transformations import euler_from_quaternion

        _, _, current_yaw = euler_from_quaternion(
            [
                current_pose.pose.orientation.x,
                current_pose.pose.orientation.y,
                current_pose.pose.orientation.z,
                current_pose.pose.orientation.w,
            ]
        )
        _, _, last_yaw = euler_from_quaternion(
            [
                last_pose.pose.orientation.x,
                last_pose.pose.orientation.y,
                last_pose.pose.orientation.z,
                last_pose.pose.orientation.w,
            ]
        )

        yaw_diff = abs(current_yaw - last_yaw)

        # 判断是否移动
        return distance > diff_distance or yaw_diff > diff_yaw
    def is_same_robot_position(
        self, last_pose: PoseStamped, current_pose: PoseStamped
    ) -> bool:
        """
        检查机器人当前位置是否与上次位置相同
        """
        if last_pose is None or current_pose is None:
            return False

        # 计算位置差异
        dx = current_pose.pose.position.x - last_pose.pose.position.x
        dy = current_pose.pose.position.y - last_pose.pose.position.y
        distance = math.sqrt(dx * dx + dy * dy)

        # 计算朝向差异
        from tf_transformations import euler_from_quaternion

        _, _, current_yaw = euler_from_quaternion(
            [
                current_pose.pose.orientation.x,
                current_pose.pose.orientation.y,
                current_pose.pose.orientation.z,
                current_pose.pose.orientation.w,
            ]
        )
        _, _, last_yaw = euler_from_quaternion(
            [
                last_pose.pose.orientation.x,
                last_pose.pose.orientation.y,
                last_pose.pose.orientation.z,
                last_pose.pose.orientation.w,
            ]
        )

        yaw_diff = abs(current_yaw - last_yaw)

        # 判断是否相同
        # return distance < 0.2 and yaw_diff < 0.2
        return distance < 0.2 
    
    def goal_fire_extinguishing_callback(self, goal_request):
        self.logger.info("Received RotateSearch goal request.")
        if self._action_goal_handle is not None and self._action_goal_handle.is_active:
            self.logger.warn(
                "Another rotation action is already active. Rejecting new goal."
            )
            return GoalResponse.REJECT
        self.logger.info("RotateSearch goal will be accepted.")
        return GoalResponse.ACCEPT

    def handle_accepted_fire_extinguishing_callback(self, goal_handle):
        self.logger.info("RotateSearch goal accepted.")
        self._action_goal_handle = goal_handle
        goal_handle.execute()

    def spin_async(self, radian: float):
        while not self.spin_client.wait_for_server(timeout_sec=1.0):
            self.logger.info("'Spin' action server not available, waiting...")
        goal_msg = Spin.Goal()
        goal_msg.target_yaw = radian
        send_goal_future = self.spin_client.send_goal_async(goal_msg)
        return send_goal_future

    def nav2pose_async(self, target_pose: PoseStamped):
        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.logger.info("'NavigateToPose' action server not available, waiting...")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose
        goal_msg.behavior_tree = ""
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        return send_goal_future

    def execute_fire_extinguishing_callback(self, goal_handle):
        self._action_goal_handle = goal_handle
        self.logger.info("Executing RotateSearch goal...")

        self.fire_extinguishing_start_time = time.time()
        result = FireExtinguishing.Result()
        feedback_msg = FireExtinguishing.Feedback()
        request = goal_handle.request

        def set_result(success: bool, message: str):
            nonlocal result
            nonlocal goal_handle
            self.logger.info(f"Setting result: success={success}, message='{message}'")
            result.success = success
            result.message = message
            if success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            self._action_goal_handle = None

        target_pose = request.target_pose
        fire_detect_result = self.nav2pose_with_fire_detection(target_pose)

        if fire_detect_result == self.FireDetectState.FireDetectionError:
            set_result(False, "Error during fire detection while navigating.")
            return result

        if not self.is_fire_detected():
            fire_detect_result = self.rotate_detect_fire(0.3)
            if fire_detect_result == self.FireDetectState.FireDetectionError:
                set_result(False, "Error during fire detection while rotating.")
                return result

        if self.fire_info.fire_position.x > 1.0:
            fire_map_pos = self.fire_info.fire_position_map
            pose, _ = self.find_best_extinguish_pose_v2(fire_map_pos.x, fire_map_pos.y)
            if pose is None:
                set_result(False, "Failed to get extinguishment pose.")
                return result
            send_goal_future = self.nav2pose_async(pose)
            rclpy.spin_until_future_complete(self, send_goal_future)

        if self.fire_info.fire_position.y > 0.2:
            fire_detect_result = self.rotate_detect_fire(0.1)

        set_result(True, "Fire extinguishing completed successfully.")
        return result

    def cancel_fire_extinguishing_callback(self, goal_handle):
        self.logger.info("Received request to cancel RotateSearch goal.")
        if not self.navigator.isTaskComplete():
            self.logger.info("Canceling active navigation task.")
        self.navigator.cancelTask()
        return CancelResponse.ACCEPT

    def rotate_detect_fire(self, rotate_speed: float=0,stop_cb=None) -> FireDetectState:
        """
        功能说明：旋转搜索火源，阻塞等待结果

        参数说明：
            rotate_speed (float): 旋转速度,如果为0，则使用默认机制，偏移大时旋转速度为0.3，偏移小时为0.1

        返回值说明：
            FireDetectState: 火源检测状态

        实现说明：
            1. 根据当前火源状态决定旋转角度
            2. 处理距离过远的情况
            3. 优化旋转逻辑
        """
        if rotate_speed != 0:
            self.motion_params_angular_max_z(rotate_speed)
        else:
            self.motion_params_angular_max_z(0.3)
        motion_params = self.motion_params_angular_max_z()
        if abs(motion_params - rotate_speed) > 0.01:  # 使用更宽松的比较
            self.logger.error(
                f"Failed to set angular_max_z to {rotate_speed}, current value is {motion_params}"
            )
            return self.FireDetectState.FireDetectionError

        # 决定旋转角度
        if self.fire_info is None:
            # self.logger.info("FireInfo is too old, rotating 360 degrees.")
            target_yaw = math.radians(360.0)
        else:
            target_yaw = math.atan(
                self.fire_info.fire_position.y / self.fire_info.fire_position.x
            )
        time_allowance=int(target_yaw/ rotate_speed + 10.0)
        if not self.navigator.spin(target_yaw,time_allowance=time_allowance):
            return self.FireDetectState.FireDetectionError
        last_move_time = time.time()
        while not self.navigator.isTaskComplete():
            last_pose = self.get_robot_pose_in_map()
            feedback = self.navigator.getFeedback()
            
            if stop_cb is not None:
                ret=stop_cb()
                if ret:
                    self.logger.info("Rotation stopped by callback.")
                    self.navigator.cancelTask()
                    return self.FireDetectState.FireDetectStop
            
            if self.is_fire_detected():
                self.logger.info("Fire detected, stopping rotation.")
                self.logger.info(f"Fire position: {self.fire_info.fire_position}")
                self.navigator.cancelTask()
                return self.FireDetectState.FireDetected
            
            # from nav2_msgs.action._spin import Spin_Feedback

            if self.is_robot_moved(last_pose, diff_distance=99, diff_yaw=0.05):
                last_move_time = time.time()
            if time.time() - last_move_time > 10.0:
                self.logger.info("Robot has not moved, backup")
                self.navigator.cancelTask()
                self._pub_cmd_vel(0.0, 0.1)
                rclpy.spin_once(self, timeout_sec=1)
                self._pub_cmd_vel(0.0, 0.0)  # 停止移动
                return self.FireDetectState.FireDetectionError

        #     if result_future.done():
        #         status = result_future.result().status
        #         if status == GoalStatus.STATUS_SUCCEEDED:
        #             self.logger.info("rotate succeeded!")
        #             return self.FireDetectState.NoFireDetected
        #         elif status == GoalStatus.STATUS_ABORTED:
        #             self.logger.error(f"rotate failed with status: {status}")
        #             return self.FireDetectState.FireDetectionError
        # send_goal_future = self.spin_async(target_yaw)

        return self.FireDetectState.NoFireDetected

    def rotate_align_fire(self,timeout_s: int=60) -> bool:
        """
        功能说明：对准火源，假设已经发现火源且在合适距离内
        """
        start_time=time.time()
        dist_x,dist_y,yaw_diff = self.get_robot_diff_from_fire_in_map()
        if dist_x is None or dist_y is None or yaw_diff is None:
            self.logger.error("Failed to get robot position or fire position.")
            return False
        if abs(yaw_diff) < 0.1:
            self.logger.info("Robot is already aligned with the fire.")
            return True
        if abs(yaw_diff) >1:
            self.motion_params_angular_max_z(0.3)
        else:
            self.motion_params_angular_max_z(0.1)

        # 决定旋转角度
        if self.fire_info is None:
            target_yaw = math.radians(360.0)
        else:
            target_yaw = math.atan(
                self.fire_info.fire_position_map.y / self.fire_info.fire_position_map.x
            )
        if not self.navigator.spin(target_yaw,time_allowance=timeout_s):
            return False
        last_move_time = time.time()
        while not self.navigator.isTaskComplete():
            if  abs(self.fire_info.fire_position.y) < 0.1:
                self.navigator.cancelTask()
                return True
            
            last_pose = self.get_robot_pose_in_map()
            if self.is_robot_moved(last_pose, diff_distance=99, diff_yaw=0.05):
                last_move_time = time.time()
            if time.time() - last_move_time > 10.0:
                self.logger.info("Robot has not moved, backup")
                self.navigator.cancelTask()
                self._pub_cmd_vel(0.0, 0.1)
                rclpy.spin_once(self, timeout_sec=1)
                self._pub_cmd_vel(0.0, 0.0)  # 停止移动
                return self.FireDetectState.FireDetectionError
            if time.time() - start_time > timeout_s:
                self.logger.error("Rotation timed out, stopping.")
                self.navigator.cancelTask()
                return False
        return False

    def nav2pose_with_fire_detection(self, target_pose: PoseStamped) -> FireDetectState:
        """移动到指定位置同时搜索火源中心，阻塞等待结果"""
        self.logger.info(
            f"Navigating to pose: x={target_pose.pose.position.x}, "
            f"y={target_pose.pose.position.y}, z={target_pose.pose.position.z}"
        )
        self.motion_params_angular_max_z(0.5)
        last_move_time = time.time()
        self.navigator.goToPose(target_pose)
        while not self.navigator.isTaskComplete():
            last_pose = self.get_robot_pose_in_map()
            feedback = self.navigator.getFeedback()
            # self.logger.info(f"{type(feedback)}") #from nav2_msgs.action._navigate_to_pose import NavigateToPose_Feedback
            if self.fire_info is None and self.is_fire_detected():
                self.logger.info("Fire detected during navigation.")
                self.logger.info(f"Fire position: {self.fire_info.fire_position}")
                self.navigator.cancelTask()
                return self.FireDetectState.FireDetected

            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=60.0):
                self.logger.error("Navigation took too long, stopping.")
                self.navigator.cancelTask()
                return self.FireDetectState.FireDetectionError

            if self.is_robot_moved(last_pose, diff_distance=0.1, diff_yaw=0.1):
                last_move_time = time.time()
            if time.time() - last_move_time > 60.0:
                self.logger.info("Robot has not moved, backup")
                self.navigator.cancelTask()
                self._pub_cmd_vel(-0.1, 0.0)  # 向后移动0.1米
                rclpy.spin_once(self, timeout_sec=1)
                self._pub_cmd_vel(0.0, 0.0)  # 停止移动
                return self.FireDetectState.FireDetectionError
        return self.FireDetectState.NoFireDetected

    def _get_fire_key(self, fire_x: float, fire_y: float) -> str:
        """生成火源位置的唯一标识"""
        return f"{fire_x:.2f}_{fire_y:.2f}"

    def _normalize_angle_0_2pi(self, angle: float) -> float:
        """将角度归一化到 [0, 2π] 范围"""
        while angle < 0:
            angle += 2.0 * math.pi
        while angle >= 2.0 * math.pi:
            angle -= 2.0 * math.pi
        return angle

    def _is_direction_excluded(
        self, fire_x: float, fire_y: float, angle: float
    ) -> bool:
        """检查指定方向是否被排除"""
        fire_key = self._get_fire_key(fire_x, fire_y)
        if fire_key not in self.excluded_directions:
            return False

        normalized_angle = self._normalize_angle_0_2pi(angle)
        excluded_ranges = self.excluded_directions[fire_key]

        # 检查当前角度是否在任何排除范围内
        for start_angle, end_angle, timestamp in excluded_ranges:
            # 检查是否过期
            if time.time() - timestamp > self.direction_exclusion_timeout:
                continue

            # 处理跨越0度的角度范围
            if start_angle <= end_angle:
                if start_angle <= normalized_angle <= end_angle:
                    return True
            else:  # 跨越0度
                if normalized_angle >= start_angle or normalized_angle <= end_angle:
                    return True

        return False

    def _exclude_direction(
        self,
        fire_x: float,
        fire_y: float,
        failed_angle: float,
        exclusion_range: float = math.pi / 6,
    ):
        """排除指定方向及其邻近范围"""
        fire_key = self._get_fire_key(fire_x, fire_y)

        if fire_key not in self.excluded_directions:
            self.excluded_directions[fire_key] = []

        # 计算排除范围
        normalized_angle = self._normalize_angle_0_2pi(failed_angle)
        start_angle = self._normalize_angle_0_2pi(
            normalized_angle - exclusion_range / 2
        )
        end_angle = self._normalize_angle_0_2pi(normalized_angle + exclusion_range / 2)

        # 添加排除范围（包含时间戳）
        exclusion_entry = (start_angle, end_angle, time.time())
        self.excluded_directions[fire_key].append(exclusion_entry)

        self.logger.info(
            f"Excluded direction range [{math.degrees(start_angle):.1f}°, {math.degrees(end_angle):.1f}°] for fire at ({fire_x:.3f}, {fire_y:.3f})"
        )

    def _cleanup_expired_exclusions(self, fire_x: float, fire_y: float):
        """清理过期的排除方向"""
        fire_key = self._get_fire_key(fire_x, fire_y)

        if fire_key not in self.excluded_directions:
            return

        current_time = time.time()
        valid_exclusions = []

        for start_angle, end_angle, timestamp in self.excluded_directions[fire_key]:
            if current_time - timestamp <= self.direction_exclusion_timeout:
                valid_exclusions.append((start_angle, end_angle, timestamp))

        if valid_exclusions:
            self.excluded_directions[fire_key] = valid_exclusions
        else:
            del self.excluded_directions[fire_key]

    def _can_navigate_to_position(self, pose: PoseStamped) -> bool:
        """检查是否可以导航到指定位置"""
        try:
            # 获取当前机器人位置
            start_pose = self.get_robot_pose_in_map()
            if start_pose is None:
                return False

            # 使用Nav2的路径规划器检查路径
            path = self.navigator.getPath(start=start_pose, goal=pose)

            if path is None or len(path.poses) < 2:
                self.logger.debug(
                    f"No path found to position ({pose.pose.position.x:.3f}, {pose.pose.position.y:.3f})"
                )
                return False

            # 检查路径长度是否合理（避免过长的绕行路径）
            path_length = 0.0
            for i in range(1, len(path.poses)):
                prev_pose = path.poses[i - 1].pose.position
                curr_pose = path.poses[i].pose.position
                path_length += math.sqrt(
                    (curr_pose.x - prev_pose.x) ** 2 + (curr_pose.y - prev_pose.y) ** 2
                )

            # 计算直线距离
            direct_distance = math.sqrt(
                (pose.pose.position.x - start_pose.pose.position.x) ** 2
                + (pose.pose.position.y - start_pose.pose.position.y) ** 2
            )

            # 如果路径长度是直线距离的3倍以上，认为路径不合理
            if path_length > direct_distance * 3.0:
                self.logger.debug(
                    f"Path too long: {path_length:.2f}m vs direct {direct_distance:.2f}m"
                )
                return False

            self.logger.debug(
                f"Valid path found: length={path_length:.2f}m, direct={direct_distance:.2f}m"
            )
            return True

        except Exception as e:
            self.logger.error(f"Error checking navigation path: {e}")
            return False

    def _can_see_fire_from_position(
        self, pose: PoseStamped, fire_x: float, fire_y: float
    ) -> bool:
        """
        检查从指定位置是否可以看到火源（改良的视线检查）
        如果火源离障碍物的距离在1m以内，认为可以到达
        """
        try:
            if not self.global_costmap:
                return True  # 如果没有costmap，假设可以看到

            # 计算从机器人位置到火源的射线
            robot_x = pose.pose.position.x
            robot_y = pose.pose.position.y

            # 使用Bresenham算法检查视线是否被阻挡
            dx = fire_x - robot_x
            dy = fire_y - robot_y
            distance = math.sqrt(dx * dx + dy * dy)

            if distance < 0.1:  # 距离太近
                return True

            # 步进检查视线
            resolution = self.global_costmap.getResolution()
            steps = int(distance / resolution)
            if steps <= 0:
                return True

            step_x = dx / steps
            step_y = dy / steps

            blocked_positions = []  # 记录被阻挡的位置

            for i in range(1, steps):
                check_x = robot_x + i * step_x
                check_y = robot_y + i * step_y

                map_coords = self.global_costmap.worldToMapValidated(check_x, check_y)
                if map_coords[0] is None:
                    continue

                mx, my = map_coords
                cost = self.global_costmap.getCostXY(mx, my)

                # 如果遇到高cost值（障碍物），记录位置
                if cost > 200:  # 可调整阈值
                    blocked_positions.append((check_x, check_y))
                    self.logger.debug(
                        f"Line of sight blocked at ({check_x:.3f}, {check_y:.3f}), cost={cost}"
                    )

            # 如果没有阻挡，直接返回True
            if not blocked_positions:
                return True

            # 检查火源是否在障碍物附近（1米以内）
            fire_near_obstacle = self._is_fire_near_obstacle(
                fire_x, fire_y, max_distance=1.0
            )

            if fire_near_obstacle:
                self.logger.debug(
                    f"Fire at ({fire_x:.3f}, {fire_y:.3f}) is near obstacle, allowing access"
                )
                return True

            # 如果视线被阻挡且火源不在障碍物附近，返回False
            self.logger.debug(f"Line of sight blocked and fire not near obstacle")
            return False

        except Exception as e:
            self.logger.error(f"Error checking line of sight: {e}")
            return True  # 出错时假设可以看到

    def _is_fire_near_obstacle(
        self, fire_x: float, fire_y: float, max_distance: float = 1.0
    ) -> bool:
        """
        检查火源是否在障碍物附近

        Args:
            fire_x, fire_y: 火源世界坐标
            max_distance: 最大距离阈值（米）

        Returns:
            True如果火源在障碍物附近，False否则
        """
        try:
            if not self.global_costmap:
                return False

            resolution = self.global_costmap.getResolution()
            search_radius_cells = int(max_distance / resolution)

            # 获取火源在地图中的坐标
            fire_coords = self.global_costmap.worldToMapValidated(fire_x, fire_y)
            if fire_coords[0] is None:
                return False

            fire_mx, fire_my = fire_coords

            # 在火源周围的圆形区域内搜索障碍物
            for dy in range(-search_radius_cells, search_radius_cells + 1):
                for dx in range(-search_radius_cells, search_radius_cells + 1):
                    # 检查是否在圆形搜索范围内
                    if dx * dx + dy * dy > search_radius_cells * search_radius_cells:
                        continue

                    check_mx = fire_mx + dx
                    check_my = fire_my + dy

                    # 检查坐标是否在地图范围内
                    if (
                        0 <= check_mx < self.global_costmap.getSizeInCellsX()
                        and 0 <= check_my < self.global_costmap.getSizeInCellsY()
                    ):

                        cost = self.global_costmap.getCostXY(check_mx, check_my)

                        # 如果发现障碍物（高cost值）
                        if cost > 200:  # 障碍物阈值
                            # 计算实际距离
                            obstacle_world = self.global_costmap.mapToWorld(
                                check_mx, check_my
                            )
                            obstacle_x, obstacle_y = obstacle_world

                            actual_distance = math.sqrt(
                                (fire_x - obstacle_x) ** 2 + (fire_y - obstacle_y) ** 2
                            )

                            if actual_distance <= max_distance:
                                self.logger.debug(
                                    f"Found obstacle at distance {actual_distance:.3f}m from fire"
                                )
                                return True

            return False

        except Exception as e:
            self.logger.error(f"Error checking fire near obstacle: {e}")
            return False

    def _can_navigate_to_position_fast(self, pose: PoseStamped) -> bool:
        """
        功能说明：快速检查是否可以导航到指定位置（简化版）

        参数说明：
            pose (PoseStamped): 目标位置

        返回值说明：
            bool: 是否可以导航

        实现说明：
            1. 使用简化的碰撞检测，避免完整路径规划
            2. 只检查目标位置的基本可达性
        """
        try:
            # 简化检查：只验证目标位置的costmap值
            target_x = pose.pose.position.x
            target_y = pose.pose.position.y

            map_coords = self.global_costmap.worldToMapValidated(target_x, target_y)
            if map_coords[0] is None:
                return False

            mx, my = map_coords
            cost = self.global_costmap.getCostXY(mx, my)

            # 简单的可达性检查
            return cost < 200  # 阈值可调整

        except Exception as e:
            self.logger.error(f"Error in fast navigation check: {e}")
            return False

    def _can_see_fire_from_position_fast(
        self, pose: PoseStamped, fire_x: float, fire_y: float
    ) -> bool:
        """
        功能说明：快速检查从指定位置是否可以看到火源（简化版）

        参数说明：
            pose (PoseStamped): 机器人位置
            fire_x (float): 火源X坐标
            fire_y (float): 火源Y坐标

        返回值说明：
            bool: 是否可以看到火源

        实现说明：
            1. 简化的视线检查，减少计算量
            2. 只检查关键点而非完整射线
        """
        try:
            robot_x = pose.pose.position.x
            robot_y = pose.pose.position.y

            # 计算距离
            distance = math.sqrt((fire_x - robot_x) ** 2 + (fire_y - robot_y) ** 2)
            if distance < 0.1:
                return True

            # 简化视线检查：只检查中点
            mid_x = (robot_x + fire_x) / 2
            mid_y = (robot_y + fire_y) / 2

            map_coords = self.global_costmap.worldToMapValidated(mid_x, mid_y)
            if map_coords[0] is None:
                return True  # 超出地图范围，假设可见

            mx, my = map_coords
            cost = self.global_costmap.getCostXY(mx, my)

            return cost < 200  # 简化的障碍物检查

        except Exception as e:
            self.logger.error(f"Error in fast vision check: {e}")
            return True

    def _get_nearest_obstacle_distance(
        self, fire_x: float, fire_y: float, search_radius: float = 2.0
    ) -> float:
        """
        获取火源到最近障碍物的距离

        Args:
            fire_x, fire_y: 火源世界坐标
            search_radius: 搜索半径（米）

        Returns:
            到最近障碍物的距离，如果没找到障碍物返回search_radius
        """
        try:
            if not self.global_costmap:
                return search_radius

            resolution = self.global_costmap.getResolution()
            search_radius_cells = int(search_radius / resolution)

            # 获取火源在地图中的坐标
            fire_coords = self.global_costmap.worldToMapValidated(fire_x, fire_y)
            if fire_coords[0] is None:
                return search_radius

            fire_mx, fire_my = fire_coords
            min_distance = search_radius

            # 在火源周围搜索障碍物
            for dy in range(-search_radius_cells, search_radius_cells + 1):
                for dx in range(-search_radius_cells, search_radius_cells + 1):
                    # 跳过中心点
                    if dx == 0 and dy == 0:
                        continue

                    # 检查是否在圆形搜索范围内
                    if dx * dx + dy * dy > search_radius_cells * search_radius_cells:
                        continue

                    check_mx = fire_mx + dx
                    check_my = fire_my + dy

                    # 检查坐标是否在地图范围内
                    if (
                        0 <= check_mx < self.global_costmap.getSizeInCellsX()
                        and 0 <= check_my < self.global_costmap.getSizeInCellsY()
                    ):

                        cost = self.global_costmap.getCostXY(check_mx, check_my)

                        # 如果发现障碍物
                        if cost > 200:
                            # 计算距离
                            obstacle_world = self.global_costmap.mapToWorld(
                                check_mx, check_my
                            )
                            obstacle_x, obstacle_y = obstacle_world

                            distance = math.sqrt(
                                (fire_x - obstacle_x) ** 2 + (fire_y - obstacle_y) ** 2
                            )

                            min_distance = min(min_distance, distance)

            return min_distance

        except Exception as e:
            self.logger.error(f"Error getting nearest obstacle distance: {e}")
            return search_radius

    def fire_obstacle_analysis_cmd(self, fire_x: str, fire_y: str) -> str:
        """调试命令：分析火源与障碍物的关系"""
        try:
            x, y = float(fire_x), float(fire_y)

            # 检查火源是否在障碍物附近
            near_obstacle = self._is_fire_near_obstacle(x, y, max_distance=1.0)

            # 获取到最近障碍物的距离
            nearest_distance = self._get_nearest_obstacle_distance(
                x, y, search_radius=2.0
            )

            # 检查火源位置的cost值
            if self.global_costmap:
                fire_coords = self.global_costmap.worldToMapValidated(x, y)
                if fire_coords[0] is not None:
                    fire_mx, fire_my = fire_coords
                    fire_cost = self.global_costmap.getCostXY(fire_mx, fire_my)
                else:
                    fire_cost = "OUT_OF_MAP"
            else:
                fire_cost = "NO_COSTMAP"

            result = [
                f"Fire Obstacle Analysis for ({x:.3f}, {y:.3f}):",
                f"  Near obstacle (≤1m): {near_obstacle}",
                f"  Nearest obstacle distance: {nearest_distance:.3f}m",
                f"  Fire position cost: {fire_cost}",
            ]

            return "\n".join(result)

        except Exception as e:
            return f"ERROR in fire obstacle analysis: {str(e)}"

    def _calculate_path_length(self, path) -> float:
        """计算路径长度"""
        if not path or len(path.poses) < 2:
            return 0.0

        total_length = 0.0
        for i in range(1, len(path.poses)):
            prev_pos = path.poses[i - 1].pose.position
            curr_pos = path.poses[i].pose.position

            dx = curr_pos.x - prev_pos.x
            dy = curr_pos.y - prev_pos.y
            total_length += math.sqrt(dx * dx + dy * dy)

        return total_length

    def _extract_yaw_from_pose(self, pose_stamped: PoseStamped) -> float:
        """从PoseStamped中提取yaw角度"""
        try:
            orientation = pose_stamped.pose.orientation
            orientation_list = [
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ]
            (_, _, yaw) = euler_from_quaternion(orientation_list)
            return yaw
        except Exception as e:
            self.logger.error(f"Error extracting yaw from pose: {e}")
            return 0.0

    def get_robot_yaw_in_map(self) -> float:
        """获取机器人在map坐标系中的yaw角度"""
        try:
            robot_pose = self.get_robot_pose_in_map()
            if robot_pose is None:
                return None

            return self._extract_yaw_from_pose(robot_pose)

        except Exception as e:
            self.logger.error(f"Error getting robot yaw: {e}")
            return None

    def find_best_extinguish_pose(self, fire_x: float, fire_y: float) -> PoseStamped:
        """
        使用官方接口高效计算最优灭火位置
        Args:
            fire_x: 火源的x坐标
            fire_y: 火源的y坐标
        Returns:
            PoseStamped: 最优灭火位置的姿态
            float: 评分，范围0-1，越高越好
        实现：以圆形每隔一定角度和距离判断footprint范围内的最大cost值
        """
        self.logger.info(
            f"Finding best extinguish pose for fire at ({fire_x:.3f}, {fire_y:.3f})"
        )

        if not self.global_costmap:
            self.logger.warn("Global costmap not available, updating...")
            self._update_costmaps()
            if not self.global_costmap:
                self.logger.error("Cannot get global costmap")
                return None

        # 参数设置
        min_distance = 0.6  # 最小距离
        max_distance = 1.2  # 最大距离
        angular_step = math.pi / 6  # 角度步长 (30度)
        radial_step = 0.1  # 径向步长

        best_pose = None
        best_score = -1.0

        start_time = time.time()

        # 在圆环内搜索候选位置
        radius = min_distance
        while radius <= max_distance:
            angle = 0.0
            while angle < 2 * math.pi:
                # 计算候选位置
                candidate_x = fire_x + radius * math.cos(angle)
                candidate_y = fire_y + radius * math.sin(angle)

                # 检查位置是否在地图范围内
                map_coords = self.global_costmap.worldToMapValidated(
                    candidate_x, candidate_y
                )
                if map_coords[0] is None:
                    angle += angular_step
                    continue

                # 计算朝向火源的角度
                yaw = math.atan2(fire_y - candidate_y, fire_x - candidate_x)

                # 使用官方碰撞检测器检查足迹碰撞
                collision_cost = self.collision_checker.footprintCostAtPose(
                    candidate_x, candidate_y, yaw, self.robot_footprint
                )
                # self.logger.info(f"collision_cost {collision_cost}")
                # 如果没有碰撞，计算评分
                if collision_cost < 253:  # 非致命障碍物
                    # 计算评分：距离理想距离越近越好，碰撞成本越低越好
                    ideal_distance = (min_distance + max_distance) / 2
                    distance_score = 1.0 - abs(radius - ideal_distance) / ideal_distance
                    collision_score = 1.0 - collision_cost / 252.0

                    total_score = distance_score * 0.3 + collision_score * 0.7

                    if total_score > best_score:
                        best_score = total_score
                        best_pose = self._create_pose_stamped(
                            candidate_x, candidate_y, yaw
                        )

                angle += angular_step
            radius += radial_step

        elapsed_time = time.time() - start_time
        self.logger.info(f"Pose search completed in {elapsed_time:.3f}s")

        if best_pose:
            self.logger.info(
                f"Best pose found: ({best_pose.pose.position.x:.3f}, {best_pose.pose.position.y:.3f}), score: {best_score:.3f}"
            )
            # 发布TF用于可视化
            self._publish_extinguish_tf(best_pose)
        else:
            self.logger.warn("No valid extinguish pose found")

        return best_pose, best_score

    def find_best_extinguish_pose_v2(self, fire_x: float, fire_y: float) -> tuple:
        """
        使用新的平均cost计算方法寻找最优灭火位置
        如果当前位置可以看到火源且无障碍物，优选当前朝向火源前0.8m
        """
        self.logger.info(
            f"Finding best extinguish pose for fire at ({fire_x:.3f}, {fire_y:.3f}) using average cost method"
        )

        if not self.global_costmap:
            self.logger.warn("Global costmap not available, updating...")
            self._update_costmaps()
            if not self.global_costmap:
                self.logger.error("Cannot get global costmap")
                return None, 0.0

        # 参数设置
        min_distance = 0.6
        max_distance = 1.2
        angular_step = math.pi / 16  # 角度步长 (11.25度)
        radial_step = 0.15  # 径向步长

        best_pose = None
        best_score = -1.0
        start_time = time.time()

        # 在圆环内搜索候选位置
        radius = min_distance
        while radius <= max_distance:
            angle = 0.0
            while angle < 2 * math.pi:
                # 计算候选位置
                candidate_x = fire_x + radius * math.cos(angle)
                candidate_y = fire_y + radius * math.sin(angle)

                # 检查位置是否在地图范围内
                map_coords = self.global_costmap.worldToMapValidated(
                    candidate_x, candidate_y
                )
                if map_coords[0] is None:
                    angle += angular_step
                    continue

                # 计算朝向火源的角度
                yaw = math.atan2(fire_y - candidate_y, fire_x - candidate_x)

                # 使用新的综合评分方法
                score = self.calculate_position_score(
                    candidate_x,
                    candidate_y,
                    yaw,
                    fire_x,
                    fire_y,
                    min_distance,
                    max_distance,
                )

                if score > best_score:
                    best_score = score
                    best_pose = self._create_pose_stamped(candidate_x, candidate_y, yaw)

                angle += angular_step
            radius += radial_step

        elapsed_time = time.time() - start_time
        self.logger.info(f"Enhanced pose search completed in {elapsed_time:.3f}s")

        if best_pose:
            self.logger.info(
                f"Best pose found: ({best_pose.pose.position.x:.3f}, {best_pose.pose.position.y:.3f}), score: {best_score:.3f}"
            )
            # 发布TF用于可视化
            self._publish_extinguish_tf(best_pose)
        else:
            self.logger.warn("No valid extinguish pose found")

        return best_pose, best_score

    def find_best_extinguish_pose_v3(self, fire_x: float, fire_y: float) -> tuple:
        """
        功能说明：使用新的平均cost计算方法寻找最优灭火位置，排除无效方向（性能优化版）

        参数说明：
            fire_x (float): 火源X坐标
            fire_y (float): 火源Y坐标

        返回值说明：
            tuple: (PoseStamped, score) 最优位置和评分

        实现说明：
            1. 限制导航检查频率，避免过度调用路径规划
            2. 增加超时机制防止死循环
            3. 优化候选位置筛选逻辑
        """
        self.logger.info(
            f"Finding best extinguish pose for fire at ({fire_x:.3f}, {fire_y:.3f}) using optimized v3 method"
        )

        # 清理过期的排除方向
        self._cleanup_expired_exclusions(fire_x, fire_y)

        if not self.global_costmap:
            self.logger.warn("Global costmap not available, updating...")
            self._update_costmaps()
            if not self.global_costmap:
                self.logger.error("Cannot get global costmap")
                return None, 0.0

        # 参数设置
        min_distance = 0.6
        max_distance = 1.2
        angular_step = math.pi / 12  # 增大角度步长 (15度)，减少候选点
        radial_step = 0.2  # 增大径向步长，减少候选点

        best_pose = None
        best_score = -1.0
        start_time = time.time()

        # 性能优化参数
        max_search_time = 10.0  # 最大搜索时间10秒
        nav_check_interval = 5  # 每5个候选点检查一次导航
        nav_check_count = 0

        # 统计信息
        total_candidates = 0
        excluded_candidates = 0
        navigation_failures = 0
        vision_failures = 0

        # 预筛选：快速costmap检查的候选位置
        valid_candidates = []

        # 在圆环内搜索候选位置
        radius = min_distance
        while radius <= max_distance:
            angle = 0.0
            while angle < 2 * math.pi:
                # 超时检查
                if time.time() - start_time > max_search_time:
                    self.logger.warn(
                        f"Search timeout after {max_search_time}s, using best found so far"
                    )
                    break

                total_candidates += 1

                # 检查该方向是否被排除
                if self._is_direction_excluded(fire_x, fire_y, angle):
                    excluded_candidates += 1
                    angle += angular_step
                    continue

                # 计算候选位置
                candidate_x = fire_x + radius * math.cos(angle)
                candidate_y = fire_y + radius * math.sin(angle)

                # 检查位置是否在地图范围内
                map_coords = self.global_costmap.worldToMapValidated(
                    candidate_x, candidate_y
                )
                if map_coords[0] is None:
                    angle += angular_step
                    continue

                # 计算朝向火源的角度
                yaw = math.atan2(fire_y - candidate_y, fire_x - candidate_x)

                # 快速costmap检查
                score = self.calculate_position_score(
                    candidate_x,
                    candidate_y,
                    yaw,
                    fire_x,
                    fire_y,
                    min_distance,
                    max_distance,
                )

                # 只对高分候选位置进行详细检查
                if score > 0.5:  # 提高阈值，减少详细检查次数
                    candidate_pose = self._create_pose_stamped(
                        candidate_x, candidate_y, yaw
                    )

                    # 限制导航检查频率
                    nav_check_count += 1
                    if nav_check_count % nav_check_interval == 0:
                        # 检查是否可以导航到该位置（简化版）
                        if not self._can_navigate_to_position_fast(candidate_pose):
                            navigation_failures += 1
                            self.logger.debug(
                                f"Cannot navigate to ({candidate_x:.3f}, {candidate_y:.3f}), excluding direction"
                            )
                            self._exclude_direction(fire_x, fire_y, angle)
                            angle += angular_step
                            continue

                        # 检查从该位置是否可以看到火源（简化版）
                        if not self._can_see_fire_from_position_fast(
                            candidate_pose, fire_x, fire_y
                        ):
                            vision_failures += 1
                            self.logger.debug(
                                f"Cannot see fire from ({candidate_x:.3f}, {candidate_y:.3f}), excluding direction"
                            )
                            self._exclude_direction(fire_x, fire_y, angle)
                            angle += angular_step
                            continue

                    # 更新最佳位置
                    if score > best_score:
                        best_score = score
                        best_pose = candidate_pose

                angle += angular_step

            radius += radial_step

            # 超时检查
            if time.time() - start_time > max_search_time:
                break

        elapsed_time = time.time() - start_time

        # 输出统计信息
        self.logger.info(f"Optimized pose search completed in {elapsed_time:.3f}s")
        self.logger.info(
            f"Search statistics: total={total_candidates}, excluded={excluded_candidates}, "
            f"nav_failures={navigation_failures}, vision_failures={vision_failures}"
        )

        if best_pose:
            self.logger.info(
                f"Best pose found: ({best_pose.pose.position.x:.3f}, {best_pose.pose.position.y:.3f}), score: {best_score:.3f}"
            )
            # 发布TF用于可视化
            self._publish_extinguish_tf(best_pose)
        else:
            self.logger.warn("No valid extinguish pose found")

        return best_pose, best_score

    #########################################调试##################################
    def report_extinguish_failure(
        self, fire_x: float, fire_y: float, failed_pose: PoseStamped, reason: str
    ):
        """报告灭火失败，排除相应方向"""
        if failed_pose is None:
            return

        # 计算失败位置相对于火源的角度
        dx = failed_pose.pose.position.x - fire_x
        dy = failed_pose.pose.position.y - fire_y
        failed_angle = math.atan2(dy, dx)

        self.logger.warn(
            f"Extinguish attempt failed at ({failed_pose.pose.position.x:.3f}, {failed_pose.pose.position.y:.3f}): {reason}"
        )

        # 排除该方向
        self._exclude_direction(
            fire_x, fire_y, failed_angle, exclusion_range=math.pi / 4
        )  # 更大的排除范围

    def clear_direction_exclusions(self, fire_x: float = None, fire_y: float = None):
        """清理方向排除（调试用）"""
        if fire_x is not None and fire_y is not None:
            fire_key = self._get_fire_key(fire_x, fire_y)
            if fire_key in self.excluded_directions:
                del self.excluded_directions[fire_key]
                self.logger.info(
                    f"Cleared exclusions for fire at ({fire_x:.3f}, {fire_y:.3f})"
                )
        else:
            self.excluded_directions.clear()
            self.failed_positions.clear()
            self.logger.info("Cleared all direction exclusions")

    def debug_exclusions_cmd(self, fire_x: str, fire_y: str) -> str:
        """调试命令：显示当前的排除方向"""
        try:
            x, y = float(fire_x), float(fire_y)
            fire_key = self._get_fire_key(x, y)

            if fire_key not in self.excluded_directions:
                return f"No exclusions for fire at ({x:.3f}, {y:.3f})"

            result = [f"Exclusions for fire at ({x:.3f}, {y:.3f}):"]
            current_time = time.time()

            for i, (start_angle, end_angle, timestamp) in enumerate(
                self.excluded_directions[fire_key]
            ):
                age = current_time - timestamp
                result.append(
                    f"  {i+1}: [{math.degrees(start_angle):.1f}°, {math.degrees(end_angle):.1f}°] (age: {age:.1f}s)"
                )

            return "\n".join(result)

        except Exception as e:
            return f"ERROR: {str(e)}"

    def find_and_approach_fire(self, fire_x: float, fire_y: float) -> bool:
        """
        功能说明：寻找火源并抵达最佳灭火位置，最终对准火源

        参数说明：
            fire_x (float): 火源的X坐标
            fire_y (float): 火源的Y坐标

        返回值说明：
            bool: True表示成功定位并对准火源，False表示失败

        实现说明：
            1. 循环尝试获取最佳灭火位置并导航
            2. 如果探测到火源则停止导航
            3. 抵达后如果没有火源则旋转搜索
            4. 找到火源后精确定位和对准
            5. 使用方向排除机制避免重复失败
        """
        self.logger.info(
            f"Starting fire search and approach for fire at ({fire_x:.3f}, {fire_y:.3f})"
        )

        def update_new_fire_position(fire_x_old, fire_y_old):
            if self.is_fire_detected() and not self.is_same_fire_point(
                fire_x_old, fire_y_old
            ):
                fire_x, fire_y = self.get_fire_position()
                self.logger.info(
                    f"Fire already detected at ({fire_x:.3f}, {fire_y:.3f}), using existing position"
                )
                return fire_x, fire_y
            else:
                self.logger.info(
                    f"Fire not detected or position changed, using provided coordinates ({fire_x_old:.3f}, {fire_y_old:.3f})"
                )
                return fire_x_old, fire_y_old

        # while True:
        #     if fire_x < 1.2:
        #         self.logger.info(
        #             f"Fire is within range ({self.fire_info.fire_position.x:.3f}m), aligning robot"
        #         )
        #         break
        #     pose, score = self.find_best_extinguish_pose_v2(fire_x, fire_y)
        #     if pose is None:
        #         self.logger.error(
        #             "No valid extinguish pose found, aborting fire search"
        #         )
        #         rclpy.spin_once(self, timeout_sec=1)
            
        #     self.logger.info(
        #         f"Best extinguish pose found: ({pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}), score: {score:.3f}"
        #     )
        #     last_pose=self.get_robot_pose_in_map()
        #     ret = self.nav2pose_with_fire_detection(pose)
        #     if ret == self.FireDetectState.FireDetected:
        #         fire_x, fire_y = update_new_fire_position(fire_x, fire_y)
        #     if self.is_same_robot_position(last_pose,self.get_robot_pose_in_map()):
        #         self.logger.info(
        #             f"Already at best extinguish pose ({pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}), aligning robot"
        #         )
        #         break
            
        def rotate_stop_cb():
            """回调函数：停止旋转"""
            if self.is_fire_detected(5) and abs(self.fire_info.fire_position.y)<0.5:
                self.motion_params_angular_max_z(0.1)
            elif self.is_fire_detected(5) and abs(self.fire_info.fire_position.y)<0.3:
                self.motion_params_angular_max_z(0.05)
                if  abs(self.fire_info.fire_position.y)<0.1:
                    self.motion_params_stop(True)
                    rclpy.spin_once(self, timeout_sec=0.5)
                    self.motion_params_stop(False)
                    self.motion_params_angular_max_z(0.5)
                    self.logger.info("Stopping rotation due to fire align")
                    return True
        while True:
            if self.is_fire_detected(5) :
                self.logger.info(
                    f"search with fire at ({fire_x:.3f}, {fire_y:.3f}), stopping search"
                )
                rclpy.spin_once(self, timeout_sec=0.5)
                break
            # 旋转搜索火源
            self.logger.info(
                f"Rotating to search with fire at ({fire_x:.3f}, {fire_y:.3f})"
            )
            ret = self.rotate_detect_fire(0.3,rotate_stop_cb)
            if ret == self.FireDetectState.FireDetected:
                fire_x, fire_y = update_new_fire_position(fire_x, fire_y)
            elif ret == self.FireDetectState.FireDetectStop:
                self.logger.info(
                    f"Stopped searching fire at ({fire_x:.3f}, {fire_y:.3f})"
                )
                return True
                
        # while True:
        #     if self.is_fire_detected() and abs(self.fire_info.fire_position.y<0.1):
        #         self.logger.info(
        #             f"Aligned with fire at ({fire_x:.3f}, {fire_y:.3f}), stopping search"
        #         )
        #         return True
        #     # 旋转搜索火源
        #     self.logger.info(
        #         f"Rotating to align with fire at ({fire_x:.3f}, {fire_y:.3f})"
        #     )
        #     rclpy.spin_once(self, timeout_sec=0.5)
        #     ret = self.rotate_detect_fire(0.1,rotate_stop_cb)
        #     if ret == self.FireDetectState.FireDetected:
        #         fire_x, fire_y = update_new_fire_position(fire_x, fire_y)
        #     rclpy.spin_once(self, timeout_sec=0.5)
            
            
    def find_and_approach_fire_cmd(self, fire_x: str, fire_y: str) -> str:
        """socket命令：寻找火源并抵达最佳灭火位置"""
        try:
            x = float(fire_x)
            y = float(fire_y)

            success = self.find_and_approach_fire(x, y)
            if success:
                return f"SUCCESS: Approached fire at ({x:.3f}, {y:.3f})"
            else:
                return "ERROR: Failed to approach fire"
        except Exception as e:
            self.logger.error(f'stacktrace: {traceback.format_exc()}')
            return f"ERROR: {str(e)}"
    # 辅助方法：清理特定火源的排除方向（调试用）
    def clear_fire_exclusions_cmd(self, fire_x: str, fire_y: str) -> str:
        """socket命令：清理指定火源的方向排除"""
        try:
            x = float(fire_x)
            y = float(fire_y)
            self.clear_direction_exclusions(x, y)
            return f"SUCCESS: Cleared exclusions for fire at ({x:.3f}, {y:.3f})"
        except Exception as e:
            return f"ERROR: {str(e)}"

    def _handle_socket_command(self, command: str) -> str:
        """
        功能说明：处理socket命令，支持自动类型转换

        参数说明：
            command (str): 接收到的命令字符串

        返回值说明：
            str: 命令执行结果

        实现说明：
            1. 解析命令和参数
            2. 查找对应的方法
            3. 自动进行参数类型转换
            4. 执行方法并返回格式化结果
        """
        parts = command.strip().split()
        if not parts:
            return "ERROR: Empty command\n"

        cmd = parts[0].lower()
        args = parts[1:]

        # 特殊命令处理
        if cmd == "ping":
            return "PONG_FROM_NAV_NODE\n"

        # 查找原方法并自动创建命令版本
        if hasattr(self, cmd):
            try:
                original_method = getattr(self, cmd)
                if callable(original_method):
                    # 自动转换参数类型
                    converted_args = auto_convert_args(original_method, args)

                    # 调用原方法
                    result = original_method(*converted_args)

                    # 格式化结果
                    formatted_result = format_method_result(result, cmd)
                    return formatted_result + "\n"
                else:
                    return f"ERROR: '{cmd}' is not callable\n"
            except Exception as e:
                return f"ERROR in {cmd}: {str(e)}\n"

        return f"ERROR: Unknown command '{cmd}'\n"


def main(args=None):
    max_retries = 3
    retry_count = 0

    while retry_count < max_retries:
        try:
            rclpy.init(args=args)
            navigation_node = NavigationNode()

            executor = MultiThreadedExecutor()
            executor.add_node(navigation_node)

            navigation_node.logger.info(
                "Navigation node and RotateSearch action server running..."
            )
            executor.spin()
            break

        except RuntimeError as e:
            retry_count += 1
            if "Nav2 initialization timeout" in str(e):
                print(
                    f"Attempt {retry_count}/{max_retries}: Nav2 initialization failed, restarting..."
                )
                try:
                    executor.shutdown()
                    navigation_node.destroy_node()
                except:
                    pass
                try:
                    rclpy.shutdown()
                except:
                    pass
                time.sleep(1)
                continue
            else:
                raise

        except KeyboardInterrupt:
            print("Keyboard interrupt, shutting down.")
            break

        finally:
            if "executor" in locals():
                executor.shutdown()
            if "navigation_node" in locals():
                navigation_node.destroy_node()
            rclpy.shutdown()

    if retry_count >= max_retries:
        print(f"Failed to initialize Nav2 after {max_retries} attempts. Exiting.")


if __name__ == "__main__":
    main()
