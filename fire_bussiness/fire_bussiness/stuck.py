def escape_stuck_situation(self, escape_distance: float = 0.5, timeout_sec: float = 15.0) -> bool:
    """
    功能说明：通过分析local costmap寻找最佳方向摆脱卡住状态
    
    参数说明：
        escape_distance (float): 逃脱移动距离，默认0.5米
        timeout_sec (float): 逃脱操作超时时间，默认15秒
        
    返回值说明：
        bool: True表示成功摆脱卡住，False表示失败
        
    实现说明：
        1. 分析local costmap找到最佳逃脱方向
        2. 在该方向上移动指定距离
        3. 检查是否成功摆脱卡住状态
    """
    try:
        self.logger.info(f"Attempting to escape stuck situation with distance {escape_distance}m")
        
        # 确保有local costmap
        if not self.local_costmap:
            self._update_costmaps()
            if not self.local_costmap:
                self.logger.error("No local costmap available for escape analysis")
                return False
        
        # 获取当前机器人位置和朝向
        current_pose = self.get_robot_pose_in_map()
        if not current_pose:
            self.logger.error("Cannot get current robot pose for escape")
            return False
        
        current_yaw = self.get_robot_yaw_in_map()
        if current_yaw is None:
            self.logger.error("Cannot get current robot yaw for escape")
            return False
        
        # 分析local costmap找到最佳逃脱方向
        best_direction = self._find_best_escape_direction(current_pose, current_yaw)
        
        if best_direction is None:
            self.logger.error("No suitable escape direction found")
            return False
        
        # 计算目标位置
        target_x = current_pose.pose.position.x + escape_distance * math.cos(best_direction)
        target_y = current_pose.pose.position.y + escape_distance * math.sin(best_direction)
        
        # 创建目标姿态（保持当前朝向）
        target_pose = self._create_pose_stamped(target_x, target_y, current_yaw)
        
        self.logger.info(f"Escaping in direction {math.degrees(best_direction):.1f}° to ({target_x:.3f}, {target_y:.3f})")
        
        # 执行逃脱移动
        success = self._execute_escape_movement(target_pose, timeout_sec)
        
        if success:
            self.logger.info("Successfully escaped stuck situation")
        else:
            self.logger.warn("Failed to escape stuck situation")
        
        return success
        
    except Exception as e:
        self.logger.error(f"Error in escape_stuck_situation: {e}")
        return False

def _find_best_escape_direction(self, current_pose: PoseStamped, current_yaw: float) -> float:
    """
    功能说明：分析local costmap找到最佳逃脱方向
    
    参数说明：
        current_pose (PoseStamped): 当前机器人位置
        current_yaw (float): 当前机器人朝向
        
    返回值说明：
        float: 最佳逃脱方向（弧度），None表示未找到合适方向
        
    实现说明：
        1. 在8个主要方向上分析costmap
        2. 计算每个方向的可行性评分
        3. 选择评分最高的方向
    """
    try:
        # 8个主要方向（每45度一个）
        candidate_directions = [
            0.0,                    # 前方
            math.pi/4,              # 右前方
            math.pi/2,              # 右侧
            3*math.pi/4,            # 右后方
            math.pi,                # 后方
            -3*math.pi/4,           # 左后方
            -math.pi/2,             # 左侧
            -math.pi/4,             # 左前方
        ]
        
        best_direction = None
        best_score = -1.0
        
        robot_x = current_pose.pose.position.x
        robot_y = current_pose.pose.position.y
        
        self.logger.debug(f"Analyzing escape directions from ({robot_x:.3f}, {robot_y:.3f})")
        
        for direction in candidate_directions:
            # 计算该方向的可行性评分
            score = self._evaluate_escape_direction(robot_x, robot_y, direction, current_yaw)
            
            direction_deg = math.degrees(direction)
            self.logger.debug(f"Direction {direction_deg:.1f}°: score = {score:.3f}")
            
            if score > best_score:
                best_score = score
                best_direction = direction
        
        if best_direction is not None:
            self.logger.info(f"Best escape direction: {math.degrees(best_direction):.1f}° (score: {best_score:.3f})")
        else:
            self.logger.warn("No suitable escape direction found")
        
        return best_direction
        
    except Exception as e:
        self.logger.error(f"Error finding best escape direction: {e}")
        return None

def _evaluate_escape_direction(self, robot_x: float, robot_y: float, direction: float, current_yaw: float, 
                              check_distance: float = 0.8) -> float:
    """
    功能说明：评估指定方向的逃脱可行性
    
    参数说明：
        robot_x, robot_y (float): 机器人当前位置
        direction (float): 待评估方向（弧度）
        current_yaw (float): 机器人当前朝向
        check_distance (float): 检查距离，默认0.8米
        
    返回值说明：
        float: 方向评分（0-1），越高越好
        
    实现说明：
        1. 沿指定方向检查路径上的障碍物
        2. 考虑方向变化的难度
        3. 综合计算评分
    """
    try:
        if not self.local_costmap:
            return 0.0
        
        # 方向优先级评分（优先选择前进方向）
        direction_preference = self._calculate_direction_preference(direction, current_yaw)
        
        # 路径清晰度评分
        path_clearness = self._calculate_path_clearness(robot_x, robot_y, direction, check_distance)
        
        # 机器人足迹安全性评分
        footprint_safety = self._calculate_footprint_safety_along_path(robot_x, robot_y, direction, check_distance)
        
        # 综合评分
        total_score = (
            direction_preference * 0.3 +    # 方向偏好30%
            path_clearness * 0.4 +          # 路径清晰度40%
            footprint_safety * 0.3          # 足迹安全性30%
        )
        
        self.logger.debug(f"Direction {math.degrees(direction):.1f}°: pref={direction_preference:.3f}, "
                         f"clear={path_clearness:.3f}, safety={footprint_safety:.3f}, total={total_score:.3f}")
        
        return total_score
        
    except Exception as e:
        self.logger.error(f"Error evaluating escape direction: {e}")
        return 0.0

def _calculate_direction_preference(self, direction: float, current_yaw: float) -> float:
    """
    功能说明：计算方向偏好评分
    
    参数说明：
        direction (float): 移动方向
        current_yaw (float): 当前朝向
        
    返回值说明：
        float: 方向偏好评分（0-1）
        
    实现说明：
        1. 前进方向评分最高
        2. 侧向移动次之
        3. 后退方向评分最低
    """
    # 计算相对于当前朝向的角度差
    angle_diff = abs(self._normalize_angle(direction - current_yaw))
    
    if angle_diff <= math.pi/4:          # 前方45度范围
        return 1.0
    elif angle_diff <= math.pi/2:        # 侧方45-90度
        return 0.8
    elif angle_diff <= 3*math.pi/4:      # 斜后方90-135度
        return 0.4
    else:                                # 后方135-180度
        return 0.2

def _calculate_path_clearness(self, start_x: float, start_y: float, direction: float, distance: float) -> float:
    """
    功能说明：计算指定方向路径的清晰度
    
    参数说明：
        start_x, start_y (float): 起始位置
        direction (float): 移动方向
        distance (float): 检查距离
        
    返回值说明：
        float: 路径清晰度评分（0-1）
        
    实现说明：
        1. 沿路径采样多个点
        2. 检查每个点的costmap值
        3. 计算平均清晰度
    """
    try:
        if not self.local_costmap:
            return 0.0
        
        # 采样参数
        num_samples = 10
        step_distance = distance / num_samples
        
        total_clearness = 0.0
        valid_samples = 0
        
        for i in range(1, num_samples + 1):
            # 计算采样点位置
            sample_distance = i * step_distance
            sample_x = start_x + sample_distance * math.cos(direction)
            sample_y = start_y + sample_distance * math.sin(direction)
            
            # 转换到local costmap坐标
            map_coords = self.local_costmap.worldToMapValidated(sample_x, sample_y)
            if map_coords[0] is None:
                continue  # 超出地图范围
            
            mx, my = map_coords
            
            # 检查是否在costmap范围内
            if (0 <= mx < self.local_costmap.getSizeInCellsX() and 
                0 <= my < self.local_costmap.getSizeInCellsY()):
                
                cost = self.local_costmap.getCostXY(mx, my)
                
                # 将cost转换为清晰度评分
                if cost < 50:           # 自由空间
                    clearness = 1.0
                elif cost < 100:        # 低成本区域
                    clearness = 0.8
                elif cost < 200:        # 中等成本区域
                    clearness = 0.4
                else:                   # 高成本/障碍物区域
                    clearness = 0.0
                
                total_clearness += clearness
                valid_samples += 1
        
        if valid_samples == 0:
            return 0.0
        
        average_clearness = total_clearness / valid_samples
        return average_clearness
        
    except Exception as e:
        self.logger.error(f"Error calculating path clearness: {e}")
        return 0.0

def _calculate_footprint_safety_along_path(self, start_x: float, start_y: float, direction: float, distance: float) -> float:
    """
    功能说明：计算沿路径的机器人足迹安全性
    
    参数说明：
        start_x, start_y (float): 起始位置
        direction (float): 移动方向
        distance (float): 检查距离
        
    返回值说明：
        float: 足迹安全性评分（0-1）
        
    实现说明：
        1. 在路径上几个关键点检查足迹碰撞
        2. 使用简化的足迹检查以提高性能
        3. 计算平均安全性
    """
    try:
        if not self.local_costmap:
            return 0.0
        
        # 检查3个关键点：25%, 50%, 75%距离处
        check_ratios = [0.25, 0.5, 0.75]
        total_safety = 0.0
        valid_checks = 0
        
        for ratio in check_ratios:
            check_distance = distance * ratio
            check_x = start_x + check_distance * math.cos(direction)
            check_y = start_y + check_distance * math.sin(direction)
            
            # 简化的足迹安全检查
            safety_score = self._simple_footprint_safety_check(check_x, check_y, direction)
            
            if safety_score >= 0:  # 有效检查
                total_safety += safety_score
                valid_checks += 1
        
        if valid_checks == 0:
            return 0.0
        
        average_safety = total_safety / valid_checks
        return average_safety
        
    except Exception as e:
        self.logger.error(f"Error calculating footprint safety: {e}")
        return 0.0

def _simple_footprint_safety_check(self, x: float, y: float, yaw: float, safety_radius: float = 0.4) -> float:
    """
    功能说明：简化的足迹安全检查
    
    参数说明：
        x, y (float): 检查位置
        yaw (float): 机器人朝向
        safety_radius (float): 安全半径，默认0.4米
        
    返回值说明：
        float: 安全性评分（0-1），-1表示检查失败
        
    实现说明：
        1. 使用圆形近似代替复杂的多边形足迹
        2. 检查圆形区域内的平均cost
        3. 返回安全性评分
    """
    try:
        if not self.local_costmap:
            return -1
        
        resolution = self.local_costmap.getResolution()
        radius_cells = int(safety_radius / resolution)
        
        # 转换到local costmap坐标
        center_coords = self.local_costmap.worldToMapValidated(x, y)
        if center_coords[0] is None:
            return -1
        
        center_mx, center_my = center_coords
        
        total_cost = 0.0
        sample_count = 0
        lethal_count = 0
        
        # 在圆形区域内采样
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                # 检查是否在圆形范围内
                if dx*dx + dy*dy <= radius_cells*radius_cells:
                    mx = center_mx + dx
                    my = center_my + dy
                    
                    # 检查是否在costmap范围内
                    if (0 <= mx < self.local_costmap.getSizeInCellsX() and 
                        0 <= my < self.local_costmap.getSizeInCellsY()):
                        
                        cost = self.local_costmap.getCostXY(mx, my)
                        total_cost += cost
                        sample_count += 1
                        
                        if cost >= 253:  # 致命障碍物
                            lethal_count += 1
        
        if sample_count == 0:
            return -1
        
        average_cost = total_cost / sample_count
        lethal_ratio = lethal_count / sample_count
        
        # 如果有致命障碍物，安全性为0
        if lethal_ratio > 0.1:  # 超过10%为致命障碍物
            return 0.0
        
        # 根据平均cost计算安全性
        if average_cost < 50:
            safety = 1.0
        elif average_cost < 100:
            safety = 0.8
        elif average_cost < 200:
            safety = 0.4
        else:
            safety = 0.1
        
        return safety
        
    except Exception as e:
        self.logger.error(f"Error in simple footprint safety check: {e}")
        return -1

def _execute_escape_movement(self, target_pose: PoseStamped, timeout_sec: float) -> bool:
    """
    功能说明：执行逃脱移动
    
    参数说明：
        target_pose (PoseStamped): 目标位置
        timeout_sec (float): 超时时间
        
    返回值说明：
        bool: 是否成功完成移动
        
    实现说明：
        1. 使用nav2进行路径规划和导航
        2. 监控移动进度
        3. 在超时时间内完成移动
    """
    try:
        self.logger.info(f"Executing escape movement to ({target_pose.pose.position.x:.3f}, {target_pose.pose.position.y:.3f})")
        
        # 记录起始位置
        start_pose = self.get_robot_pose_in_map()
        if not start_pose:
            return False
        
        # 发送导航目标
        send_goal_future = self.nav2pose_async(target_pose)
        
        # 等待目标被接受
        start_time = time.time()
        goal_accept_timeout = 5.0
        
        while time.time() - start_time < goal_accept_timeout:
            rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=0.1)
            
            if send_goal_future.done():
                try:
                    goal_result = send_goal_future.result()
                    if goal_result.accepted:
                        self.logger.info("Escape navigation goal accepted")
                        break
                    else:
                        self.logger.error("Escape navigation goal rejected")
                        return False
                except Exception as e:
                    self.logger.error(f"Error getting escape navigation result: {e}")
                    return False
        else:
            self.logger.error("Escape navigation goal acceptance timeout")
            return False
        
        # 执行导航
        result_future = goal_result.get_result_async()
        
        while rclpy.ok() and (time.time() - start_time) < timeout_sec:
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=0.1)
            
            if result_future.done():
                try:
                    result = result_future.result()
                    status = result.status
                    
                    if status == GoalStatus.STATUS_SUCCEEDED:
                        self.logger.info("Escape movement completed successfully")
                        return True
                    else:
                        self.logger.error(f"Escape movement failed with status: {status}")
                        return False
                        
                except Exception as e:
                    self.logger.error(f"Error getting escape movement result: {e}")
                    return False
        
        # 超时处理
        self.logger.warn("Escape movement timeout, canceling...")
        try:
            cancel_future = goal_result.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
        except Exception as e:
            self.logger.error(f"Error canceling escape movement: {e}")
        
        return False
        
    except Exception as e:
        self.logger.error(f"Error executing escape movement: {e}")
        return False

# 添加socket命令接口
def escape_stuck_cmd(self, distance: str = "0.5", timeout: str = "15") -> str:
    """socket命令：摆脱卡住状态"""
    try:
        escape_distance = float(distance)
        timeout_sec = float(timeout)
        
        success = self.escape_stuck_situation(escape_distance, timeout_sec)
        
        if success:
            return f"SUCCESS: Escaped stuck situation by moving {escape_distance}m"
        else:
            return f"FAILED: Could not escape stuck situation"
            
    except Exception as e:
        return f"ERROR: {str(e)}"

# 添加检测卡住状态的方法
def is_robot_stuck(self, check_duration: float = 5.0, min_movement: float = 0.1) -> bool:
    """
    功能说明：检测机器人是否卡住
    
    参数说明：
        check_duration (float): 检查持续时间，默认5秒
        min_movement (float): 最小移动距离，默认0.1米
        
    返回值说明：
        bool: True表示机器人卡住，False表示正常
        
    实现说明：
        1. 记录初始位置
        2. 等待指定时间
        3. 检查位置变化
    """
    try:
        # 获取初始位置
        start_pose = self.get_robot_pose_in_map()
        if not start_pose:
            return False
        
        start_time = time.time()
        
        # 等待检查时间
        while time.time() - start_time < check_duration:
            time.sleep(0.5)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # 获取结束位置
        end_pose = self.get_robot_pose_in_map()
        if not end_pose:
            return False
        
        # 计算移动距离
        dx = end_pose.pose.position.x - start_pose.pose.position.x
        dy = end_pose.pose.position.y - start_pose.pose.position.y
        movement = math.sqrt(dx*dx + dy*dy)
        
        is_stuck = movement < min_movement
        
        if is_stuck:
            self.logger.warn(f"Robot appears stuck: only moved {movement:.3f}m in {check_duration}s")
        else:
            self.logger.info(f"Robot moving normally: moved {movement:.3f}m in {check_duration}s")
        
        return is_stuck
        
    except Exception as e:
        self.logger.error(f"Error checking if robot stuck: {e}")
        return False

def check_stuck_cmd(self, duration: str = "5.0", min_move: str = "0.1") -> str:
    """socket命令：检查机器人是否卡住"""
    try:
        check_duration = float(duration)
        min_movement = float(min_move)
        
        is_stuck = self.is_robot_stuck(check_duration, min_movement)
        
        if is_stuck:
            return f"WARNING: Robot appears to be stuck (moved < {min_movement}m in {check_duration}s)"
        else:
            return f"OK: Robot is moving normally"
            
    except Exception as e:
        return f"ERROR: {str(e)}"