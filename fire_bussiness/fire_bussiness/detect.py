#! /usr/bin/env python3
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy, tf2_ros
import tf2_geometry_msgs

# from rclpy.duration import Duration
from builtin_interfaces.msg import Duration
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
from .socket_shell import TCPSocketServer  # Import the new class

from fire_interfaces.msg import FireInfo
from action_msgs.msg import GoalStatus

'''
class FireCaculate:
    """
    一个容器用来存储odom、火源坐标、火源温度
    原计划用一个相机通过移动进行3角定位，现在不需要了
    """

    def __init__(self, logger, max_temperature=55.0):
        self.logger = logger
        self.container = []  # odom,FireInfo
        self.max_temperature = max_temperature  # 最大温度阈值
        # 可见光
        self.color_hfov = math.radians(104.8)
        self.color_vfov = math.radians(55.8)
        # 热成像
        self.ir_hfov = math.radians(50.0)
        self.ir_vfov = math.radians(37.2)

        # self.fire_detected = False
        # self.detected_time = time.time()  # 记录检测到火源的时间
        self.detected_distance = 0.0  # 火源距离机器人最近的距离

    def add_fire_info(self, odom: Odometry, fire_info: FireInfo):
        """
        添加火源信息到容器中
        :param odom: Odometry 消息
        :param fire_info: FireInfo 消息
        """
        if fire_info.max <= self.max_temperature:
            return
        self.container.append((odom, fire_info))

    def is_fire_now(self):
        """
        检查当前是否有火源
        :return: 如果有火源返回True，否则返回False
        """
        if not self.container:
            return False

        # 检查容器中是否有火源温度超过最大温度阈值
        for _, fire_info in self.container:
            if fire_info.max > self.max_temperature:
                return True
        return False

    def get_last_fire_info(self):
        """
        获取最后一次检测到的火源信息
        :return: 最后一次检测到的火源信息 (Odometry, FireInfo)
        """
        if not self.container:
            return None, None

        return self.container[-1]

    def cuculate_rotate_degrees(self, current_postion: Odometry):
        """
        计算旋转到目标火源位置需要的角度
        :param target_fire_position: 目标火源位置
        :return: 旋转到目标火源位置需要的角度（弧度）
        """
        if not self.container:
            return 0.0

        last_odom, fire_info = self.get_last_fire_info()
        if last_odom is None:
            return 0.0

        # 获取当前机器人的朝向
        orientation_q = last_odom.pose.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        ]
        (_, _, last_odom_yaw) = euler_from_quaternion(orientation_list)
        current_q = current_postion.pose.pose.orientation
        current_orientation_list = [current_q.x, current_q.y, current_q.z, current_q.w]
        (_, _, current_yaw) = euler_from_quaternion(current_orientation_list)
        # TODO 把相机的tf转换为odom的tf
        fire_diff_yaw = (fire_info.center_x - 0.5) * self.ir_hfov
        self.logger.info(
            f"last_odom_yaw: {last_odom_yaw}, fire_diff_yaw: {fire_diff_yaw}, current_yaw: {current_yaw}"
        )
        delta_yaw = last_odom_yaw - fire_diff_yaw - current_yaw

        return delta_yaw

    def caculate_fire_distace(self):
        """
        计算火源距离机器人最近的距离
        :return: 最近的火源距离
        """
        if not self.container:
            return 0.0

        return 0.0
'''


class NavigationNode(Node):
    from enum import Enum

    class FireDetectState(Enum):
        FireDetected = 1
        NoFireDetected = 2
        FireDetectionError = 3

    def __init__(self):
        super().__init__("navigation_node")
        self.logger = ColorLogger(self)
        self.logger.info("Initializing NavigationNode...")
        self.navigator = BasicNavigator()
        self.goal_active = False
        self._action_goal_handle = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.tcp_server_host = "0.0.0.0"  # Listen on all interfaces
        self.tcp_server_port = 12345  # Choose a port
        self.tcp_socket_server = TCPSocketServer(
            host=self.tcp_server_host,
            port=self.tcp_server_port,
            command_handler_callback=self._handle_socket_command,  # Your existing handler
            logger=self.get_logger(),  # Use the ROS node's logger
            rclpy_ok_check=rclpy.ok,  # Pass rclpy.ok directly
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

        # self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.current_odom = None
        self.feedback = None
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )
        self.current_yaw = 0.0
        # self.initial_yaw_direct_rotation = 0.0
        # self.total_rotated_yaw_direct = 0.0

        self.motion_params_client = AsyncParameterClient(self, "/motion_jetson")

        self.set_initial_pose_service = self.create_service(
            SetInitialPoseSrv, "set_initial_pose", self.handle_set_initial_pose
        )
        self.set_navigation_goal_service = self.create_service(
            SetNavigationGoalSrv, "set_navigation_goal", self.handle_set_navigation_goal
        )

        self.nav_cb_group = ReentrantCallbackGroup()
        # Action Server for RotateSearch
        self.fire_extinguishing_action_server = ActionServer(
            self,
            FireExtinguishing,
            "FireExtinguishing",
            execute_callback=self.execute_fire_extinguishing_callback,  # 实际执行
            goal_callback=self.goal_fire_extinguishing_callback,  # 用于防止重复提交请求
            handle_accepted_callback=self.handle_accepted_fire_extinguishing_callback,  # 保存请求状态
            cancel_callback=self.cancel_fire_extinguishing_callback,  # 取消导航和旋转
            callback_group=self.nav_cb_group,
        )
        self.fire_extinguishing_start_time = time.time()
        # self.fire_extinguishing_timeout_timer = self.create_timer(1, self.check_fire_extinguishing_timeout)

        self.nav_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose", callback_group=self.nav_cb_group
        )

        self.spin_client = ActionClient(
            self, Spin, "spin", callback_group=self.nav_cb_group
        )
        self.logger.info("RotateSearch action server started.")

        self.logger.info("Navigation node started. Waiting for Nav2 to be active...")
        self.navigator.waitUntilNav2Active(localizer="robot_localization")
        # TODO 添加cartographer定位节点的判断，包括当前定位置信度
        self.nav2_initialized = True
        self.nav2_init_timer.cancel()  # Cancel the initialization timer
        self.logger.info("Nav2 is active.")
        # self.nav_status_timer = self.create_timer(1.0, self.check_navigation_status)

    def fire_info_callback(self, msg: FireInfo):
        # self.logger.info(f"Received fire info: {msg.fire_position}")
        self.fire_info = msg
        # self.fire_caculator.add_fire_info(self.current_odom, msg)

    def is_fire_detected(self):
        return self.fire_info and time.time() - self.fire_info.header.stamp.sec < 1

    def check_nav2_init_timeout(self):
        """Check if Nav2 initialization has timed out"""
        return
        if self.nav2_initialized:
            # Nav2 initialized successfully, cancel the timer
            self.nav2_init_timer.cancel()
            return
        self.logger.info("Checking Nav2 initialization timeout...")
        elapsed_time = time.time() - self.nav2_init_start_time
        if elapsed_time > 10.0:  # 10 seconds timeout
            self.logger.error(
                "Nav2 initialization timed out after 10 seconds. Restarting node..."
            )
            # Cancel the initialization timer
            self.nav2_init_timer.cancel()
            # Request node restart
            self.destroy_node()
            # Raise an exception to trigger node restart in main
            raise RuntimeError("Nav2 initialization timeout")

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

    def odom_callback(self, msg: Odometry):
        self.current_odom = msg
        orientation_q = msg.pose.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        ]
        (_, _, yaw) = euler_from_quaternion(orientation_list)

        # if self._action_goal_handle is not None and self._action_goal_handle.is_active:
        #     delta_yaw = yaw - self.current_yaw
        #     if delta_yaw > math.pi:
        #         delta_yaw -= 2 * math.pi
        #     elif delta_yaw < -math.pi:
        #         delta_yaw += 2 * math.pi
        #     self.total_rotated_yaw_direct += delta_yaw

        self.current_yaw = yaw

    def get_robot_pose_in_map(self):
        # 构造 odom 下的 PoseStamped
        odom_pose = PoseStamped()
        odom_pose.header.frame_id = "odom"
        # odom_pose.header.stamp = self.current_odom.header.stamp
        odom_pose.header.stamp = rclpy.time.Time(seconds=0)
        odom_pose.pose = self.current_odom.pose.pose

        try:
            # 转换到 map 坐标系
            map_pose = self.tf_buffer.transform(
                odom_pose, "map", timeout=rclpy.duration.Duration(seconds=0.5)
            )
            return map_pose.pose.position
        except Exception as e:
            self.logger.error(f"TF transform odom->map failed: {e}")
            return None

    def get_extinguishment_pose(self):
        """
        计算以当前机器人朝向为准，火源前0.8m的位置
        """
        if self.current_odom is None or self.fire_info is None:
            self.logger.error("No odom or fire info available.")
            return None

        # 1. 获取机器人和火源在map下的坐标
        robot_pos = self.get_robot_pose_in_map()
        fire_pos = self.fire_info.fire_position_map
        if robot_pos is None or fire_pos is None:
            self.logger.error("No valid robot or fire position in map frame.")
            return None
        self.logger.info(
            f"Robot position: ({robot_pos.x:.2f}, {robot_pos.y:.2f}), "
            f"Fire position: ({fire_pos.x:.2f}, {fire_pos.y:.2f})"
        )
        # 2. 计算机器人到火源的方向向量
        dx = fire_pos.x - robot_pos.x
        dy = fire_pos.y - robot_pos.y
        dist = math.hypot(dx, dy)
        if dist < 0.5:
            self.logger.error("Robot and fire are at the same position!")
            return None

        # 3. 单位化方向向量
        ux = dx / dist
        uy = dy / dist

        # 4. 计算目标点（火源前0.8m，沿机器人->火源方向，离火源0.8m）
        extinguish_x = fire_pos.x - ux * 0.8
        extinguish_y = fire_pos.y - uy * 0.8

        # 5. 目标点朝向（面向火源）
        yaw = math.atan2(dy, dx)
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        self.logger.info(
            f"Extinguishment position: ({extinguish_x:.2f}, {extinguish_y:.2f}), "
            f"Yaw: {math.degrees(yaw):.1f}°"
        )
        # 6. 构造PoseStamped
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = extinguish_x
        pose.pose.position.y = extinguish_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.logger.info(
            f"Extinguishment pose: ({extinguish_x:.2f}, {extinguish_y:.2f}), yaw: {math.degrees(yaw):.1f}°"
        )
        return pose

    def _feedbackCallback(self, msg):
        self.logger.info(f"Received action feedback message {msg}")
        self.feedback = msg.feedback
        return

    def goal_fire_extinguishing_callback(self, goal_request):
        self.logger.info("Received RotateSearch goal request.")
        if self._action_goal_handle is not None and self._action_goal_handle.is_active:
            self.logger.warn(
                "Another rotation action is already active. Rejecting new goal."
            )
            return GoalResponse.REJECT
        self.logger.info("RotateSearch goal will be accepted.")  # New log for clarity
        return GoalResponse.ACCEPT

    def handle_accepted_fire_extinguishing_callback(
        self, goal_handle
    ):  # ENTIRE METHOD REMOVED or COMMENTED OUT
        self.logger.info("RotateSearch goal accepted.")
        self._action_goal_handle = goal_handle
        goal_handle.execute()

    def spin_async(self, radian: float):
        while not self.spin_client.wait_for_server(timeout_sec=1.0):
            self.logger.info("'Spin' action server not available, waiting...")
        goal_msg = Spin.Goal()
        goal_msg.target_yaw = radian  # Spin 360 degrees
        send_goal_future = self.spin_client.send_goal_async(
            goal_msg,
        )
        return send_goal_future

    def nav2pose_async(self, target_pose: PoseStamped):
        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.logger.info("'NavigateToPose' action server not available, waiting...")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose
        goal_msg.behavior_tree = ""
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
        )
        return send_goal_future

    def execute_fire_extinguishing_callback(self, goal_handle):
        # Assign _action_goal_handle here
        self._action_goal_handle = goal_handle
        self.logger.info("Executing RotateSearch goal...")

        self.fire_extinguishing_start_time = time.time()
        result = FireExtinguishing.Result()
        feedback_msg = FireExtinguishing.Feedback()
        request = goal_handle.request  # fire_interfaces.action.FireExtinguishing_Goal

        # self.logger.info(f"Received FireExtinguishing request: {request}")
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
        fire_detect_result = self.nav2pose_with_fire_detection(target_pose, True)

        if fire_detect_result == self.FireDetectState.FireDetectionError:
            set_result(False, "Error during fire detection while navigating.")
            return result

        if not self.is_fire_detected():
            fire_detect_result = self.rotate_detect_fire(0.3)
            if fire_detect_result == self.FireDetectState.FireDetectionError:
                set_result(False, "Error during fire detection while rotating.")
                return result

        if self.fire_info.fire_position.x > 1.0:
            pose = self.get_extinguishment_pose()
            if pose is None:
                set_result(False, "Failed to get extinguishment pose.")
                return result
            send_goal_future = self.nav2pose_async(pose)
            rclpy.spin_until_future_complete(self, send_goal_future)
        # feedback_msg.current_state = FireExtinguishing.Feedback.STATE_SCANNING
        # feedback_msg.current_pose.pose = self.current_odom.pose.pose
        # feedback_msg.current_pose.header.stamp = self.current_odom.header.stamp
        # goal_handle.publish_feedback(feedback_msg)
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

    def rotate_detect_fire(self, rotate_speed: float) -> FireDetectState:
        """
        旋转搜索火源，阻塞等待结果
        """
        self.motion_params_angular_max_z(rotate_speed)
        motion_params = self.motion_params_angular_max_z()
        if motion_params != rotate_speed:
            self.logger.error(
                f"Failed to set angular_max_z to 0.1, current value is {motion_params}"
            )
            return False

        # 检测时间差，如何self.fire_info和当前时间差距1s，则绕一圈
        if self.fire_info is None or time.time() - self.fire_info.header.stamp.sec > 1:
            self.logger.info("FireInfo is too old, rotating 360 degrees.")
            target_yaw = math.radians(360.0)
        else:
            target_yaw = math.atan(
                self.fire_info.fire_position.y / self.fire_info.fire_position.x
            )
        send_goal_future = self.spin_async(target_yaw)
        while True:
            if send_goal_future.done():
                try:
                    goal_result = send_goal_future.result()
                except Exception as e:
                    self.logger.error(f"Failed to rotate: {e}")
                    return False

                if not goal_result.accepted:
                    self.logger.error("rotate was rejected!")
                    return False
                result_future = goal_result.get_result_async()
                self.logger.info("spin start.")
                break
            self.logger.info("Waiting for goal accepted")
            self.logger.info(f"state:{send_goal_future._state}")
            rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=0.1)

        while rclpy.ok():
            if self.is_fire_detected():
                self.logger.info("Fire detected, stopping rotation.")
                self.logger.info(f"Fire position: {self.fire_info.fire_position}")
                goal_result.cancel_goal_async()
                return self.FireDetectState.FireDetected
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=0.1)
            if result_future.done():
                status = result_future.result().status
                if status == GoalStatus.STATUS_SUCCEEDED:
                    self.logger.info("rotate succeeded!")
                    return self.FireDetectState.NoFireDetected
                elif status == GoalStatus.STATUS_ABORTED:
                    self.logger.error(f"rotate failed with status: {status}")
                    return self.FireDetectState.FireDetectionError
            self.logger.info("Waiting for rotate to complete...")

    def nav2pose_with_fire_detection(
        self, target_pose: PoseStamped, stop_on_fire_detected: bool
    ) -> FireDetectState:
        """
        移动到指定位置同时搜索火源中心，阻塞等待结果
        """
        self.logger.info(
            f"Navigating to pose: x={target_pose.pose.position.x}, "
            f"y={target_pose.pose.position.y}, z={target_pose.pose.position.z}"
        )
        self.motion_params_angular_max_z(0.5)
        send_goal_future = self.nav2pose_async(target_pose)
        rclpy.spin_until_future_complete(self, send_goal_future)

        while True:
            if send_goal_future.done():
                try:
                    goal_result = send_goal_future.result()
                except Exception as e:
                    self.logger.error(f"Failed to navigate: {e}")
                    return False

                if not goal_result.accepted:
                    self.logger.error("navigate was rejected!")
                    return False
                result_future = goal_result.get_result_async()
                self.logger.info("navigate start.")
                break
            self.logger.info("Waiting for goal accepted")
            self.logger.info(f"state:{send_goal_future._state}")

            rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=0.1)

        while rclpy.ok():
            if stop_on_fire_detected and self.is_fire_detected():
                self.logger.info("Fire detected, stopping navigation.")
                self.logger.info(f"Fire position: {self.fire_info.fire_position}")
                goal_result.cancel_goal_async()
                return self.FireDetectState.FireDetected
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=0.1)
            if result_future.done():
                status = result_future.result().status
                if status == GoalStatus.STATUS_SUCCEEDED:
                    self.logger.info("navigation succeeded!")
                    return self.FireDetectState.NoFireDetected
                elif status == GoalStatus.STATUS_ABORTED:
                    self.logger.error(f"navigation failed with status: {status}")
                    return self.FireDetectState.FireDetectionError
            self.logger.info("Waiting for navigation to complete...")

    def nav2extinguishment_pose(self):
        """
        逐步移动到灭火位置，不停检查和火源的距离，知道0.5m
        TODO 地形可能很复杂，需要结合多种方法导航
        通过nav2的cmd_vel进行移动并自动避障
        通过nav2导航到火源前方，但是可能无法到达
        需要判断着火位置前方有没有障碍物，可以尝试通过pgm或local costmap地图判断
        """
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        fire_detect_result = self.nav2pose_with_fire_detection(pose, True)

        if fire_detect_result == self.FireDetectState.FireDetectionError:
            self.logger.error("Error during fire detection while navigating.")
            return
        self.logger.info("Navigation to init pose completed.")
        if not self.is_fire_detected():
            fire_detect_result = self.rotate_detect_fire(0.3)
            if fire_detect_result == self.FireDetectState.FireDetectionError:
                self.logger.error("Error during fire detection while rotating.")
                return
        self.logger.info("Fire detected, proceeding to extinguishment pose.")
        
        while abs(self.fire_info.fire_position.x) > 1.0:
            time.sleep(0.5)
            pose = self.get_extinguishment_pose()
            if pose is None:
                self.logger.error("Failed to get extinguishment pose.")
                return
            fire_detect_result = self.nav2pose_with_fire_detection(pose, False)
            if fire_detect_result == self.FireDetectState.FireDetectionError:
                self.logger.error(
                    "Error during fire detection while navigating to extinguishment pose."
                )
                return
        self.logger.info("Moving towards extinguishment pose.")
        while abs(self.fire_info.fire_position.y) > 0.2:
            fire_detect_result = self.rotate_detect_fire(0.1)
            if fire_detect_result == self.FireDetectState.FireDetectionError:
                self.logger.error(
                    "Error during fire detection while rotating to extinguishment pose."
                )
                return
        self.logger.info(f"Extinguishment pose reached: {self.fire_info.fire_position}")
        # while self.fire_info.fire_position.x > 0.5:
        #     # Check if the robot is close enough to the fire
        #     if self.fire_info.fire_position.x < 0.5:
        #         self.logger.info(
        #             "Robot is close enough to the fire, stopping navigation."
        #         )
        #         break

        #     # Continue moving towards the fire
        #     self.navigator.driveOnHeading(dist=0.1, speed=0.2, time_allowance=10)
        #     time.sleep(1)

    def check_fire_extinguishing_timeout(self):
        if self._action_goal_handle is None or not self._action_goal_handle.is_active:
            return

        elapsed_seconds = time.time() - self.fire_extinguishing_start_time

        if elapsed_seconds > self._action_goal_handle.request.timeout_seconds:
            self.logger.warn(
                f"FireExtinguishing action timed out after {self._action_goal_handle.request.timeout_seconds} seconds."
            )
            self._action_goal_handle.abort()
            # self.nav_client.cancel_goal_async()
            # self.spin_client.cancel_goal_async()
            result = FireExtinguishing.Result()
            result.success = False
            result.message = "Action timed out."
            self._action_goal_handle = None
            return result

    def rotate_with_nav2_spin(self, target_spin_degrees=360.0):
        self.logger.info(f"Starting Nav2 Spin: {target_spin_degrees} degrees.")
        target_spin_rad = math.radians(target_spin_degrees)
        spin_success = self.navigator.spin(spin_dist=target_spin_rad)
        if spin_success:
            self.logger.info("Nav2 Spin completed successfully.")
        else:
            self.logger.error("Nav2 Spin failed or was canceled.")
        return spin_success

    def handle_set_initial_pose(self, request, response):
        self.logger.info("Received request to set initial pose.")
        try:
            clean_initial_pose = PoseStamped()
            clean_initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            clean_initial_pose.header.frame_id = "map"

            clean_initial_pose.pose.position.x = (
                request.initial_pose.pose.pose.position.x
            )
            clean_initial_pose.pose.position.y = (
                request.initial_pose.pose.pose.position.y
            )
            clean_initial_pose.pose.position.z = (
                request.initial_pose.pose.pose.position.z
            )
            clean_initial_pose.pose.orientation.x = (
                request.initial_pose.pose.pose.orientation.x
            )
            clean_initial_pose.pose.orientation.y = (
                request.initial_pose.pose.pose.orientation.y
            )
            clean_initial_pose.pose.orientation.z = (
                request.initial_pose.pose.pose.orientation.z
            )
            clean_initial_pose.pose.orientation.w = (
                request.initial_pose.pose.pose.orientation.w
            )

            self.navigator.setInitialPose(clean_initial_pose)
            response.success = True
            response.message = "Initial pose set successfully."
            self.logger.info("Initial pose set.")
        except Exception as e:
            response.success = False
            response.message = f"Failed to set initial pose: {str(e)}"
            self.logger.error(f"Error in set_initial_pose: {str(e)}")
        return response

    def handle_set_navigation_goal(self, request, response):
        self.logger.info("Received request to set navigation goal.")
        if self.goal_active:
            response.success = False
            response.message = "A goal is already active. Please cancel or wait."
            self.logger.warn("Attempted to set new goal while current goal is active.")
            return response
        try:
            goal_pose_msg = request.goal_pose
            goal_pose_msg.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose_msg.header.frame_id = "map"

            self.logger.info(
                f"Navigating to goal: x={goal_pose_msg.pose.position.x}, "
                f"y={goal_pose_msg.pose.position.y}"
            )
            self.navigator.goToPose(goal_pose_msg)
            self.goal_active = True
            response.success = True
            response.message = "Navigation goal accepted."
        except Exception as e:
            response.success = False
            response.message = f"Failed to set navigation goal: {str(e)}"
            self.logger.error(f"Error in set_navigation_goal: {str(e)}")
        return response

    def check_navigation_status(self):
        if not self.goal_active:
            return

    def print_fire_info(self):
        """
        Print the current fire information.
        """
        if self.fire_info is None:
            self.logger.info("No fire information available.")
            return

        self.logger.info(
            f"Fire Info: Position: ({self.fire_info.fire_position.x}, "
            f"{self.fire_info.fire_position.y}), "
            f"time: {self.fire_info.header.stamp.sec}.{self.fire_info.header.stamp.nanosec}"
        )

    def spin_async_cmd(self, radian: str):
        self.spin_async(float(radian))

    def _handle_socket_command(self, command: str) -> str:
        """
        Process commands received via socket.
        Returns a string response.
        """
        parts = command.lower().split()
        if not parts:
            return "ERROR: Empty command\n"

        cmd = parts[0]
        args = parts[1:]

        if cmd == "ping":
            return "PONG_FROM_NAV_NODE\n"
        elif cmd == "spin_async":
            self.spin_async(float(args[0]) if args else 360.0)
        elif hasattr(self, cmd):
            # Check if the command is a method of this class
            method = getattr(self, cmd, None)
            if callable(method):
                try:
                    # Call the method with arguments if any
                    if args:
                        return str(method(*args))
                    else:
                        return str(method())
                except Exception as e:
                    return f"ERROR: {str(e)}\n"
            else:
                return f"ERROR: Command '{cmd}' is not callable or does not exist\n"
        else:
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
            break  # If we get here, everything is working correctly

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
                time.sleep(1)  # Wait a bit before retrying
                continue
            else:
                raise  # Re-raise other RuntimeErrors

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
