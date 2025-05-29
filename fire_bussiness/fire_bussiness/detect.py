#! /usr/bin/env python3
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
# from rclpy.duration import Duration
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from rclpy.action import ActionServer,ActionClient, CancelResponse, GoalResponse
from nav2_msgs.action import  Spin
from nav2_msgs.action import (
    NavigateToPose,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter_client import AsyncParameterClient
from rclpy.parameter import Parameter
from fire_interfaces.srv import SetInitialPoseSrv, SetNavigationGoalSrv
from fire_interfaces.action import RotateSearch,FireExtinguishing
import math
from tf_transformations import euler_from_quaternion
import time
from .log import ColorLogger
from .socket_shell import TCPSocketServer # Import the new class

from fire_interfaces.msg import FireInfo
from action_msgs.msg import GoalStatus
import socket
import threading
import os # For socket cleanup

class FireCaculate():
    '''
    一个容器用来存储odom、火源坐标、火源温度
    '''
    def __init__(self,logger, max_temperature=55.0):
        self.logger=logger
        self.container=[] #odom,FireInfo
        self.max_temperature = max_temperature  # 最大温度阈值
        #可见光
        self.color_hfov=math.radians(104.8)
        self.color_vfov=math.radians(55.8)
        #热成像
        self.ir_hfov=math.radians(50.0)
        self.ir_vfov=math.radians(37.2)
        
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
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, last_odom_yaw) = euler_from_quaternion(orientation_list)
        current_q=current_postion.pose.pose.orientation
        current_orientation_list = [current_q.x, current_q.y, current_q.z, current_q.w]
        (_, _, current_yaw) = euler_from_quaternion(current_orientation_list)
        #TODO 把相机的tf转换为odom的tf
        fire_diff_yaw=(fire_info.center_x - 0.5)*self.ir_hfov
        self.logger.info(f'last_odom_yaw: {last_odom_yaw}, fire_diff_yaw: {fire_diff_yaw}, current_yaw: {current_yaw}')
        delta_yaw = last_odom_yaw -fire_diff_yaw - current_yaw
        
        return delta_yaw
    def caculate_fire_distace(self):
        """
        计算火源距离机器人最近的距离
        :return: 最近的火源距离
        """
        if not self.container:
            return 0.0
        
        return 0.0

class NavigationNode(Node):
    
    def __init__(self):
        super().__init__('navigation_node')
        self.logger=ColorLogger(self)
        self.logger.info("Initializing NavigationNode...")
        self.navigator = BasicNavigator()
        self.goal_active = False 
        self._action_goal_handle = None 
        
        
        self.tcp_server_host = "0.0.0.0"  # Listen on all interfaces
        self.tcp_server_port = 12345      # Choose a port
        self.tcp_socket_server = TCPSocketServer(
            host=self.tcp_server_host,
            port=self.tcp_server_port,
            command_handler_callback=self._handle_socket_command, # Your existing handler
            logger=self.get_logger(), # Use the ROS node's logger
            rclpy_ok_check=rclpy.ok # Pass rclpy.ok directly
        )
        if not self.tcp_socket_server.start():
            self.logger.error("Failed to start TCP socket server during node initialization.")
        
        
        self.fire_caculator = FireCaculate(self.logger, 40.0)
        self.fire_info_sub= self.create_subscription(
                FireInfo,
                '/hk_camera/fire_info',
                self.fire_info_callback,
                10
            )
        
        self.nav2_init_start_time = time.time()
        self.nav2_init_timer = self.create_timer(1, self.check_nav2_init_timeout)
        self.nav2_initialized = False
        

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.current_odom =None
        self.feedback = None
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.current_yaw = 0.0
        self.initial_yaw_direct_rotation = 0.0 
        self.total_rotated_yaw_direct = 0.0    

        self.motion_params_client = AsyncParameterClient(self, '/motion_jetson')

        self.set_initial_pose_service = self.create_service(
            SetInitialPoseSrv, 'set_initial_pose', self.handle_set_initial_pose)
        self.set_navigation_goal_service = self.create_service(
            SetNavigationGoalSrv, 'set_navigation_goal', self.handle_set_navigation_goal)

        self.nav_cb_group = ReentrantCallbackGroup()
        # Action Server for RotateSearch
        self.fire_extinguishing_action_server = ActionServer(
            self,
            FireExtinguishing,
            'FireExtinguishing',
            execute_callback=self.execute_fire_extinguishing_callback, #实际执行
            goal_callback=self.goal_fire_extinguishing_callback,  #用于防止重复提交请求
            handle_accepted_callback=self.handle_accepted_fire_extinguishing_callback, #保存请求状态
            cancel_callback=self.cancel_fire_extinguishing_callback,  #取消导航和旋转
            callback_group=self.nav_cb_group,
        )
        self.fire_extinguishing_start_time = time.time()
        # self.fire_extinguishing_timeout_timer = self.create_timer(1, self.check_fire_extinguishing_timeout)

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.nav_cb_group
        )
        
        self.spin_client = ActionClient(
            self,
            Spin,
            'spin',
            callback_group=self.nav_cb_group
        )
        self.logger.info("RotateSearch action server started.")

        self.logger.info('Navigation node started. Waiting for Nav2 to be active...')
        self.navigator.waitUntilNav2Active()
        self.nav2_initialized = True
        self.nav2_init_timer.cancel()  # Cancel the initialization timer
        self.logger.info('Nav2 is active.')
        # self.nav_status_timer = self.create_timer(1.0, self.check_navigation_status)

    def fire_info_callback(self, msg: FireInfo):
        self.logger.debug(f"Received fire info: {msg.center_x} "  f"{msg.center_y}, "
                         f"Temperature: {msg.max}")
        
        self.fire_caculator.add_fire_info(self.current_odom, msg)
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
            self.logger.error("Nav2 initialization timed out after 10 seconds. Restarting node...")
            # Cancel the initialization timer
            self.nav2_init_timer.cancel()
            # Request node restart
            self.destroy_node()
            # Raise an exception to trigger node restart in main
            raise RuntimeError("Nav2 initialization timeout")

    def motion_params_angular_max_z(self, value=None):
        if value is not None:
            self.logger.info(f"Setting angular_max_z to {value}")
            future = self.motion_params_client.set_parameters([
                Parameter('angular_max_z', Parameter.Type.DOUBLE, value)
            ])
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            if  result:
                self.logger.info("Parameter set successfully.")
            else:
                self.logger.error("Failed to set parameter.")
        else:
            future = self.motion_params_client.get_parameters(['angular_max_z'])
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
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        
        if self._action_goal_handle is not None and self._action_goal_handle.is_active:
            delta_yaw = yaw - self.current_yaw 
            if delta_yaw > math.pi:
                delta_yaw -= 2 * math.pi
            elif delta_yaw < -math.pi:
                delta_yaw += 2 * math.pi
            self.total_rotated_yaw_direct += delta_yaw
        
        self.current_yaw = yaw
        
    def _feedbackCallback(self, msg):
        self.logger.info(f'Received action feedback message {msg}')
        self.feedback = msg.feedback
        return
    
    def goal_fire_extinguishing_callback(self, goal_request):
        self.logger.info('Received RotateSearch goal request.')
        if self._action_goal_handle is not None and self._action_goal_handle.is_active:
            self.logger.warn('Another rotation action is already active. Rejecting new goal.')
            return GoalResponse.REJECT
        self.logger.info('RotateSearch goal will be accepted.') # New log for clarity
        return GoalResponse.ACCEPT

    def handle_accepted_fire_extinguishing_callback(self, goal_handle): # ENTIRE METHOD REMOVED or COMMENTED OUT
        self.logger.info('RotateSearch goal accepted.')
        self._action_goal_handle = goal_handle
        goal_handle.execute()

    def spin_async(self,radian: float):
        while not self.spin_client.wait_for_server(timeout_sec=1.0):
            self.logger.info("'Spin' action server not available, waiting...")
        goal_msg = Spin.Goal()
        goal_msg.target_yaw = math.radians(radian)  # Spin 360 degrees
        send_goal_future = self.spin_client.send_goal_async(
            goal_msg, 
        )
        return send_goal_future
    
    def nav2pose_async(self, target_pose: PoseStamped):
        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.logger.info("'NavigateToPose' action server not available, waiting...")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose
        goal_msg.behavior_tree = ''
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg, 
        )
        return send_goal_future

    def execute_fire_extinguishing_callback(self, goal_handle):
        # Assign _action_goal_handle here
        self._action_goal_handle = goal_handle
        self.logger.info('Executing RotateSearch goal...')
        
        self.rotate_aim_fire_center(None)
        return None
        
        self.fire_extinguishing_start_time = time.time()
        result = FireExtinguishing.Result()
        feedback_msg = FireExtinguishing.Feedback()
        request = goal_handle.request #fire_interfaces.action.FireExtinguishing_Goal
        # self.logger.info(f"Received FireExtinguishing request: {request}")
        target_pose = request.target_pose
        
        self.logger.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose
        goal_msg.behavior_tree = ''

        self.logger.info(
            'Navigating to goal: '
            + str(goal_msg.pose.pose.position.x)
            + ' '
            + str(goal_msg.pose.pose.position.y)
            + '...'
        )
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg, self._feedbackCallback
        )

        while True:
            if send_goal_future.done():
                try:
                    goal_result = send_goal_future.result()
                except Exception as e:
                    self.logger.error(f"Failed to send goal: {e}")
                    result.success = False
                    result.message = f"Failed to send goal: {str(e)}"
                    self._action_goal_handle = None 
                    return result
                
                # First check if the goal was accepted
                if not goal_result.accepted:
                    self.logger.error(
                        f'Goal to {goal_msg.pose.pose.position.x} {goal_msg.pose.pose.position.y} was rejected!'
                    )
                    result.success = False
                    result.message = "Goal was rejected."
                    self._action_goal_handle = None 
                    return result
                result_future = goal_result.get_result_async()
                break  # Exit the while loop once we've gotten the goal handle
            # While waiting for the future, publish feedback
            feedback_msg.current_state = FireExtinguishing.Feedback.STATE_NAVIGATING
            feedback_msg.current_pose.pose = self.current_odom.pose.pose
            feedback_msg.current_pose.header.stamp = self.current_odom.header.stamp
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)

        # Now monitor the navigation progress
        while rclpy.ok():
            if result_future.done():
                status = result_future.result().status
                if status == GoalStatus.STATUS_SUCCEEDED:
                    self.logger.info('Navigation succeeded!')
                    break
                else:
                    self.logger.error(f'Navigation failed with status: {status}')
                    result.success = False
                    result.message = f"Navigation failed with status: {status}"
                    self._action_goal_handle = None 
                    return result

            # Continue publishing feedback while waiting
            feedback_msg.current_state = FireExtinguishing.Feedback.STATE_NAVIGATING
            feedback_msg.current_pose.pose = self.current_odom.pose.pose
            feedback_msg.current_pose.header.stamp = self.current_odom.header.stamp
            goal_handle.publish_feedback(feedback_msg)
            if self.fire_caculator.is_fire_now():
                self.logger.info("Fire detected during navigation, stopping navigation.")
                self.navigator.cancelTask()
                goal_handle.succeed()
                result.success = False
                result.message = "Fire detected during navigation."
                self._action_goal_handle = None 
                return result
            time.sleep(0.1)
            
        self.logger.debug("Waiting for 'Spin' action server")
        while not self.spin_client.wait_for_server(timeout_sec=1.0):
            self.logger.info("'Spin' action server not available, waiting...")
        goal_msg = Spin.Goal()
        goal_msg.target_yaw = math.radians(360.0)  # Spin 360 degrees
        # time_allowance =time.time() - self.fire_extinguishing_start_time
        # if time_allowance <= 0:
        #     time_allowance = 10  # Ensure a minimum time allowance
        # goal_msg.time_allowance = Duration(sec=time_allowance)

        send_goal_future = self.spin_client.send_goal_async(
            goal_msg, self._feedbackCallback
        )
        while True:
            if send_goal_future.done():
                try:
                    goal_result = send_goal_future.result()
                except Exception as e:
                    self.logger.error(f"Failed to rotate: {e}")
                    result.success = False
                    result.message = f"Failed to rotate: {str(e)}"
                    self._action_goal_handle = None 
                    return result
                
                # First check if the goal was accepted
                if not goal_result.accepted:
                    self.logger.error(  f'rotate was rejected!')
                    result.success = False
                    result.message = "rotate was rejected."
                    self._action_goal_handle = None 
                    return result
                result_future = goal_result.get_result_async()
                break  # Exit the while loop once we've gotten the goal handle
            # While waiting for the future, publish feedback
            feedback_msg.current_state = FireExtinguishing.Feedback.STATE_SCANNING
            feedback_msg.current_pose.pose = self.current_odom.pose.pose
            feedback_msg.current_pose.header.stamp = self.current_odom.header.stamp
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)

        # Now monitor the navigation progress
        while rclpy.ok():
            if result_future.done():
                status = result_future.result().status
                if status == GoalStatus.STATUS_SUCCEEDED:
                    self.logger.info('rotate succeeded!')
                    break
                else:
                    self.logger.error(f'rotate failed with status: {status}')
                    result.success = False
                    result.message = f"rotate failed with status: {status}"
                    self._action_goal_handle = None 
                    return result

            # Continue publishing feedback while waiting
            feedback_msg.current_state = FireExtinguishing.Feedback.STATE_SCANNING
            feedback_msg.current_pose.pose = self.current_odom.pose.pose
            feedback_msg.current_pose.header.stamp = self.current_odom.header.stamp
            goal_handle.publish_feedback(feedback_msg)
            if self.fire_caculator.is_fire_now():
                self.logger.info("Fire detected during rotate, stopping rotate.")
                self.spin_client.cancelTask()
                goal_handle.succeed()
                result.success = False
                result.message = "Fire detected during rotate."
                self._action_goal_handle = None 
                return result
            time.sleep(0.1)
        
        goal_handle.succeed()  # Mark the goal as succeeded
        result.success = False
        result.message = "No fire detect."
        self._action_goal_handle = None
        return result

    def cancel_fire_extinguishing_callback(self, goal_handle):
        self.logger.info('Received request to cancel RotateSearch goal.')
        if not self.navigator.isTaskComplete():
            self.logger.info('Canceling active navigation task.')
        self.navigator.cancelTask()
        return CancelResponse.ACCEPT
    
    def rotate_aim_fire_center(self, target_fire_position: PoseStamped):
        rotate_speed = 0.05  # Default rotation speed
        self.motion_params_angular_max_z(rotate_speed)
        motion_params = self.motion_params_angular_max_z()
        if motion_params != rotate_speed:
            self.logger.error(f"Failed to set angular_max_z to 0.1, current value is {motion_params}")
            return False
        
        self.logger.debug("Waiting for 'Spin' action server")
        while not self.spin_client.wait_for_server(timeout_sec=1.0):
            self.logger.warn("'Spin' action server not available, waiting...")
        goal_msg = Spin.Goal()
        goal_msg.target_yaw = math.radians(360.0)  # TODO 根据odom计算
        self.logger.info('spin send goal.')
        send_goal_future = self.spin_client.send_goal_async(
            goal_msg, self._feedbackCallback
        )
        spin_done_flag = False
        def spin_done_cb(future):
            nonlocal spin_done_flag
            if future.done():
                spin_done_flag = True
        send_goal_future.add_done_callback(spin_done_cb)
        while True:
            # if send_goal_future.done():
            #     try:
            #         goal_result = send_goal_future.result()
            #     except Exception as e:
            #         self.logger.error(f"Failed to rotate: {e}")
            #         return False

            #     if not goal_result.accepted:
            #         self.logger.error(  f'rotate was rejected!')
            #         return False
            #     result_future = goal_result.get_result_async()
            #     self.logger.info('spin start.')
            #     break  
            if spin_done_flag:
                self.logger.info('spin done.')
                break
            if self.fire_caculator.is_fire_now() :
                _, fire_info = self.fire_caculator.get_last_fire_info()
                if fire_info.center_x <0.6 and fire_info.center_y > 0.4:
                    self.logger.info("aim_successed, fire detected during rotate, stopping rotate.")
                    self.spin_client.cancelTask()
                    return True
                else:
                    self.logger.info("Fire detected during rotate, but not in aim position, continue rotating.")
            self.logger.debug("Waiting for rotate to complete...")
            
            time.sleep(0.1)

        while rclpy.ok():
            if result_future.done():
                status = result_future.result().status
                if status == GoalStatus.STATUS_SUCCEEDED:
                    self.logger.info('rotate succeeded!')
                    break
                elif status == GoalStatus.STATUS_ABORTED:
                    self.logger.error(f'rotate failed with status: {status}')
                    return False
            if self.fire_caculator.is_fire_now() :
                _, fire_info = self.fire_caculator.get_last_fire_info()
                if fire_info.center_x <0.6 and fire_info.center_y > 0.4:
                    self.logger.info("aim_successed, fire detected during rotate, stopping rotate.")
                    self.spin_client.cancelTask()
                    return True
                else:
                    self.logger.info("Fire detected during rotate, but not in aim position, continue rotating.")
            self.logger.debug("Waiting for rotate to complete...")
            time.sleep(0.1)
        
    def check_fire_extinguishing_timeout(self):
        if self._action_goal_handle is None or not self._action_goal_handle.is_active:
            return
        
        elapsed_seconds = time.time() - self.fire_extinguishing_start_time
        
        if elapsed_seconds > self._action_goal_handle.request.timeout_seconds:
            self.logger.warn(f'FireExtinguishing action timed out after {self._action_goal_handle.request.timeout_seconds} seconds.')
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
        self.logger.info('Received request to set initial pose.')
        try:
            clean_initial_pose = PoseStamped() 
            clean_initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            clean_initial_pose.header.frame_id = 'map'
            
            clean_initial_pose.pose.position.x = request.initial_pose.pose.pose.position.x
            clean_initial_pose.pose.position.y = request.initial_pose.pose.pose.position.y
            clean_initial_pose.pose.position.z = request.initial_pose.pose.pose.position.z
            clean_initial_pose.pose.orientation.x = request.initial_pose.pose.pose.orientation.x
            clean_initial_pose.pose.orientation.y = request.initial_pose.pose.pose.orientation.y
            clean_initial_pose.pose.orientation.z = request.initial_pose.pose.pose.orientation.z
            clean_initial_pose.pose.orientation.w = request.initial_pose.pose.pose.orientation.w

            self.navigator.setInitialPose(clean_initial_pose)
            response.success = True
            response.message = 'Initial pose set successfully.'
            self.logger.info('Initial pose set.')
        except Exception as e:
            response.success = False
            response.message = f'Failed to set initial pose: {str(e)}'
            self.logger.error(f'Error in set_initial_pose: {str(e)}')
        return response

    def handle_set_navigation_goal(self, request, response):
        self.logger.info('Received request to set navigation goal.')
        if self.goal_active:
            response.success = False
            response.message = 'A goal is already active. Please cancel or wait.'
            self.logger.warn('Attempted to set new goal while current goal is active.')
            return response
        try:
            goal_pose_msg = request.goal_pose
            goal_pose_msg.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose_msg.header.frame_id = 'map' 

            self.logger.info(f'Navigating to goal: x={goal_pose_msg.pose.position.x}, '
                                   f'y={goal_pose_msg.pose.position.y}')
            self.navigator.goToPose(goal_pose_msg)
            self.goal_active = True
            response.success = True
            response.message = 'Navigation goal accepted.'
        except Exception as e:
            response.success = False
            response.message = f'Failed to set navigation goal: {str(e)}'
            self.logger.error(f'Error in set_navigation_goal: {str(e)}')
        return response

    def check_navigation_status(self):
        if not self.goal_active:
            return
        
    def print_cuculate_rotate_degrees(self):
        yaw=self.fire_caculator.cuculate_rotate_degrees(self.current_odom)
        self.spin_async(yaw)
        self.logger.info(f"current odom {self.current_odom.pose.pose}")
        fire_odom,fire_info = self.fire_caculator.get_last_fire_info()
        self.logger.info(f"fire info {fire_info}")
        self.logger.info(f"fire odom {fire_odom.pose.pose}")
        self.logger.info(f"Calculated rotation degrees: {math.degrees(yaw)} degrees")
        
    def print_fire_container(self):
        if not self.fire_caculator.container:
            self.logger.info("No fire information in the container.")
            return
        
        for odom, fire_info in self.fire_caculator.container:
            self.logger.info(f"Fire Info: {fire_info}, Odom: {odom.pose.pose}")
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
            
            navigation_node.logger.info("Navigation node and RotateSearch action server running...")
            executor.spin()
            break  # If we get here, everything is working correctly
            
        except RuntimeError as e:
            retry_count += 1
            if "Nav2 initialization timeout" in str(e):
                print(f"Attempt {retry_count}/{max_retries}: Nav2 initialization failed, restarting...")
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
            if 'executor' in locals():
                executor.shutdown()
            if 'navigation_node' in locals():
                navigation_node.destroy_node()
            rclpy.shutdown()
    
    if retry_count >= max_retries:
        print(f"Failed to initialize Nav2 after {max_retries} attempts. Exiting.")

if __name__ == '__main__':
    main()