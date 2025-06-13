def find_best_extinguish_pose_v2(self, fire_x: float, fire_y: float) -> tuple:
    """
    使用新的平均cost计算方法寻找最优灭火位置
    增加当前位置优先逻辑：如果当前位置可以看到火源且无障碍物，优选当前朝向火源前0.8m
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

    # 首先检查当前位置是否满足条件
    current_pose = self.get_robot_pose_in_map()
    if current_pose:
        optimal_pose = self._check_current_position_for_fire(current_pose, fire_x, fire_y)
        if optimal_pose:
            self.logger.info("Using optimized position from current robot location")
            # 发布TF用于可视化
            self._publish_extinguish_tf(optimal_pose)
            return optimal_pose, 1.0  # 给最高评分

    # 如果当前位置不满足条件，继续原有的搜索逻辑
    self.logger.info("Current position not optimal, searching alternative positions...")

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

def _check_current_position_for_fire(self, current_pose: PoseStamped, fire_x: float, fire_y: float) -> PoseStamped:
    """
    功能说明：检查当前位置是否适合作为灭火起点，如果适合则返回优化后的位置
    
    参数说明：
        current_pose (PoseStamped): 当前机器人位置
        fire_x (float): 火源X坐标
        fire_y (float): 火源Y坐标
        
    返回值说明：
        PoseStamped: 优化后的灭火位置，如果当前位置不适合则返回None
        
    实现说明：
        1. 检查当前位置到火源的视线是否清晰
        2. 检查当前位置到火源的距离是否合适
        3. 如果条件满足，计算朝向火源前0.8米的最优位置
    """
    try:
        current_x = current_pose.pose.position.x
        current_y = current_pose.pose.position.y
        current_yaw = self._extract_yaw_from_pose(current_pose)
        
        # 计算当前位置到火源的距离
        distance_to_fire = math.sqrt((fire_x - current_x)**2 + (fire_y - current_y)**2)
        
        self.logger.debug(f"Current distance to fire: {distance_to_fire:.3f}m")
        
        # 检查距离是否在合理范围内（不能太近也不能太远）
        if distance_to_fire < 1.0 or distance_to_fire > 3.0:
            self.logger.debug(f"Current distance to fire ({distance_to_fire:.3f}m) not suitable")
            return None
        
        # 检查当前位置到火源的视线是否清晰
        if not self._can_see_fire_from_position(current_pose, fire_x, fire_y):
            self.logger.debug("Cannot see fire from current position")
            return None
        
        # 检查当前位置的通行性
        current_score = self.calculate_position_score(
            current_x, current_y, current_yaw, fire_x, fire_y, 0.6, 1.2
        )
        
        if current_score < 0.3:  # 当前位置评分太低
            self.logger.debug(f"Current position score too low: {current_score:.3f}")
            return None
        
        # 计算朝向火源的最优位置（火源前0.8米）
        optimal_distance = 0.8
        
        # 计算从火源朝向当前位置的方向向量
        direction_x = current_x - fire_x
        direction_y = current_y - fire_y
        direction_length = math.sqrt(direction_x**2 + direction_y**2)
        
        if direction_length < 0.1:  # 避免除零
            self.logger.debug("Too close to fire source")
            return None
        
        # 标准化方向向量
        unit_x = direction_x / direction_length
        unit_y = direction_y / direction_length
        
        # 计算最优位置（火源前0.8米）
        optimal_x = fire_x + optimal_distance * unit_x
        optimal_y = fire_y + optimal_distance * unit_y
        
        # 计算朝向火源的角度
        optimal_yaw = math.atan2(fire_y - optimal_y, fire_x - optimal_x)
        
        # 检查最优位置是否在地图范围内
        map_coords = self.global_costmap.worldToMapValidated(optimal_x, optimal_y)
        if map_coords[0] is None:
            self.logger.debug("Optimal position outside map bounds")
            return None
        
        # 检查最优位置的通行性
        optimal_score = self.calculate_position_score(
            optimal_x, optimal_y, optimal_yaw, fire_x, fire_y, 0.6, 1.2
        )
        
        if optimal_score < 0.5:  # 最优位置评分太低
            self.logger.debug(f"Optimal position score too low: {optimal_score:.3f}")
            return None
        
        # 检查从最优位置到火源的视线
        optimal_pose = self._create_pose_stamped(optimal_x, optimal_y, optimal_yaw)
        if not self._can_see_fire_from_position(optimal_pose, fire_x, fire_y):
            self.logger.debug("Cannot see fire from optimal position")
            return None
        
        # 检查从当前位置到最优位置的路径
        if not self._is_path_clear(current_pose, optimal_pose):
            self.logger.debug("Path to optimal position is blocked")
            return None
        
        self.logger.info(
            f"Optimal fire position found at ({optimal_x:.3f}, {optimal_y:.3f}), "
            f"distance to fire: {optimal_distance:.3f}m, score: {optimal_score:.3f}"
        )
        
        return optimal_pose
        
    except Exception as e:
        self.logger.error(f"Error checking current position for fire: {e}")
        return None

def _is_path_clear(self, start_pose: PoseStamped, end_pose: PoseStamped) -> bool:
    """
    功能说明：检查两点之间的路径是否清晰
    
    参数说明：
        start_pose (PoseStamped): 起始位置
        end_pose (PoseStamped): 结束位置
        
    返回值说明：
        bool: True表示路径清晰，False表示有障碍物
        
    实现说明：
        1. 使用直线路径检查
        2. 沿路径采样多个点检查障碍物
        3. 考虑机器人足迹大小
    """
    try:
        if not self.global_costmap:
            return True
        
        start_x = start_pose.pose.position.x
        start_y = start_pose.pose.position.y
        end_x = end_pose.pose.position.x
        end_y = end_pose.pose.position.y
        
        # 计算路径长度和方向
        dx = end_x - start_x
        dy = end_y - start_y
        path_length = math.sqrt(dx**2 + dy**2)
        
        if path_length < 0.1:  # 距离太近
            return True
        
        # 沿路径采样检查
        resolution = self.global_costmap.getResolution()
        num_samples = max(5, int(path_length / resolution))
        
        for i in range(1, num_samples):
            ratio = i / num_samples
            check_x = start_x + ratio * dx
            check_y = start_y + ratio * dy
            
            # 检查该点的可通行性（使用简化的足迹检查）
            if not self._is_position_traversable(check_x, check_y):
                self.logger.debug(f"Path blocked at ({check_x:.3f}, {check_y:.3f})")
                return False
        
        return True
        
    except Exception as e:
        self.logger.error(f"Error checking path clearance: {e}")
        return True  # 出错时假设路径清晰

def _is_position_traversable(self, x: float, y: float, safety_margin: float = 0.3) -> bool:
    """
    功能说明：检查指定位置是否可通行
    
    参数说明：
        x, y (float): 世界坐标位置
        safety_margin (float): 安全边距，默认0.3米
        
    返回值说明：
        bool: True表示可通行，False表示有障碍物
        
    实现说明：
        1. 检查指定位置周围的costmap值
        2. 考虑机器人尺寸的安全边距
        3. 使用简化的圆形检查
    """
    try:
        if not self.global_costmap:
            return True
        
        resolution = self.global_costmap.getResolution()
        margin_cells = int(safety_margin / resolution)
        
        # 转换到地图坐标
        map_coords = self.global_costmap.worldToMapValidated(x, y)
        if map_coords[0] is None:
            return False
        
        center_mx, center_my = map_coords
        
        # 在安全边距内检查障碍物
        for dy in range(-margin_cells, margin_cells + 1):
            for dx in range(-margin_cells, margin_cells + 1):
                # 圆形检查
                if dx**2 + dy**2 <= margin_cells**2:
                    check_mx = center_mx + dx
                    check_my = center_my + dy
                    
                    # 检查是否在地图范围内
                    if (0 <= check_mx < self.global_costmap.getSizeInCellsX() and 
                        0 <= check_my < self.global_costmap.getSizeInCellsY()):
                        
                        cost = self.global_costmap.getCostXY(check_mx, check_my)
                        
                        # 如果发现高cost区域（障碍物）
                        if cost > 200:  # 可调整阈值
                            return False
        
        return True
        
    except Exception as e:
        self.logger.error(f"Error checking position traversability: {e}")
        return True

# 添加调试命令
def check_current_fire_position_cmd(self, fire_x: str, fire_y: str) -> str:
    """socket命令：检查当前位置对于指定火源的适用性"""
    try:
        x, y = float(fire_x), float(fire_y)
        
        current_pose = self.get_robot_pose_in_map()
        if not current_pose:
            return "ERROR: Cannot get current robot pose"
        
        optimal_pose = self._check_current_position_for_fire(current_pose, x, y)
        
        if optimal_pose:
            return f"SUCCESS: Optimal position found at ({optimal_pose.pose.position.x:.3f}, {optimal_pose.pose.position.y:.3f})"
        else:
            return f"INFO: Current position not optimal for fire at ({x:.3f}, {y:.3f})"
            
    except Exception as e:
        return f"ERROR: {str(e)}"