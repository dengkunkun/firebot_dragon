    class RegionScanner:
        """区域扫描器 - 实现智能的覆盖扫描"""
        
        def __init__(self, nav_node):
            self.nav_node = nav_node
            self.logger = nav_node.logger
            
            # 相机参数
            self.camera_fov = math.radians(50)  # 水平视场角
            self.camera_range = 10.0  # 探测距离
            
            # 采样参数
            self.sample_resolution = 0.1  # 采样分辨率 0.1m
            
            # 扫描状态
            self.coverage_map = None  # 覆盖地图
            self.scan_points = []     # 采样点列表
            self.scanned_points = set()  # 已扫描点集合
            self.scan_poses = []      # 扫描姿态列表
            
        def initialize_coverage_map(self, scan_region_bounds=None):
            """
            初始化覆盖地图
            
            Args:
                scan_region_bounds: (min_x, min_y, max_x, max_y) 扫描区域边界，单位m，None则使用整个costmap
            """
            try:
                if not self.nav_node.global_costmap:
                    self.logger.error("Global costmap not available for coverage mapping")
                    return False
                    
                costmap = self.nav_node.global_costmap
                
                # 获取地图边界
                if scan_region_bounds is None:
                    # 使用整个地图
                    map_width = costmap.getSizeInCellsX()
                    map_height = costmap.getSizeInCellsY()
                    
                    min_corner = costmap.mapToWorld(0, 0)
                    max_corner = costmap.mapToWorld(map_width-1, map_height-1)
                    
                    min_x, min_y = min_corner
                    max_x, max_y = max_corner
                else:
                    min_x, min_y, max_x, max_y = scan_region_bounds
                
                self.logger.info(f"Initializing coverage map for region: ({min_x:.1f}, {min_y:.1f}) to ({max_x:.1f}, {max_y:.1f})")
                
                # 生成采样网格
                self.scan_points = []
                x = min_x
                while x <= max_x:
                    y = min_y
                    while y <= max_y:
                        # 检查该点是否在可通行区域（不在障碍物上）
                        if self._is_point_scannable(x, y):
                            self.scan_points.append((x, y))
                        y += self.sample_resolution
                    x += self.sample_resolution
                
                # 初始化覆盖地图（所有点都未扫描）
                self.scanned_points = set()
                
                self.logger.info(f"Generated {len(self.scan_points)} scan points with resolution {self.sample_resolution}m")
                return True
                
            except Exception as e:
                self.logger.error(f"Error initializing coverage map: {e}")
                return False

        def _is_point_scannable(self, x: float, y: float) -> bool:
            """
            检查点是否可以被扫描（不在障碍物内）
            是否在边界外，cost < 100
            Args: 单位m
            """
            try:
                if not self.nav_node.global_costmap:
                    return False
                    
                map_coords = self.nav_node.global_costmap.worldToMapValidated(x, y)
                if map_coords[0] is None:
                    return False  # 超出地图范围
                    
                mx, my = map_coords
                cost = self.nav_node.global_costmap.getCostXY(mx, my)
                
                # 只有在自由空间或低cost区域才认为可扫描
                return cost < 100
                
            except Exception as e:
                return False

        def calculate_scan_coverage(self, robot_x: float, robot_y: float, robot_yaw: float) -> list:
            """
            计算机器人当前姿态下的扫描覆盖范围
            
            Args:
                robot_x, robot_y: 机器人位置
                robot_yaw: 机器人朝向
                
            Returns:
                被覆盖的点列表 [(x, y), ...]
            """
            covered_points = []
            
            try:
                # 计算扫描扇形的边界角度
                left_angle = robot_yaw - self.camera_fov / 2
                right_angle = robot_yaw + self.camera_fov / 2
                
                # 检查每个采样点是否在扫描范围内
                for point_x, point_y in self.scan_points:
                    # 计算点到机器人的距离和角度
                    dx = point_x - robot_x
                    dy = point_y - robot_y
                    distance = math.sqrt(dx*dx + dy*dy)
                    
                    # 距离检查
                    if distance > self.camera_range:
                        continue
                        
                    # 角度检查
                    point_angle = math.atan2(dy, dx)
                    
                    # 处理角度wrap-around
                    angle_diff = self._angle_difference(point_angle, robot_yaw)
                    if abs(angle_diff) <= self.camera_fov / 2:
                        # 检查视线是否被阻挡
                        if self._has_clear_line_of_sight(robot_x, robot_y, point_x, point_y):
                            covered_points.append((point_x, point_y))
                
                return covered_points
                
            except Exception as e:
                self.logger.error(f"Error calculating scan coverage: {e}")
                return []

        def _angle_difference(self, angle1: float, angle2: float) -> float:
            """计算两个角度之间的最小差值"""
            diff = angle1 - angle2
            while diff > math.pi:
                diff -= 2 * math.pi
            while diff < -math.pi:
                diff += 2 * math.pi
            return diff

        def _has_clear_line_of_sight(self, x1: float, y1: float, x2: float, y2: float) -> bool:
            """检查两点之间是否有清晰的视线 检查两个点之间点的cost值"""
            try:
                if not self.nav_node.global_costmap:
                    return True
                    
                # 使用简单的射线检测
                dx = x2 - x1
                dy = y2 - y1
                distance = math.sqrt(dx*dx + dy*dy)
                
                if distance < 0.01:
                    return True
                    
                # 步进检查
                steps = int(distance / (self.nav_node.global_costmap.getResolution() * 2))
                if steps <= 0:
                    return True
                    
                step_x = dx / steps
                step_y = dy / steps
                
                for i in range(1, steps):
                    check_x = x1 + i * step_x
                    check_y = y1 + i * step_y
                    
                    map_coords = self.nav_node.global_costmap.worldToMapValidated(check_x, check_y)
                    if map_coords[0] is None:
                        continue
                        
                    mx, my = map_coords
                    cost = self.nav_node.global_costmap.getCostXY(mx, my)
                    
                    # 如果遇到障碍物，视线被阻挡
                    if cost > 150:
                        return False
                        
                return True
                
            except Exception as e:
                return False

        def update_scan_coverage(self, robot_x: float, robot_y: float, robot_yaw: float):
            """更新扫描覆盖状态"""
            covered_points = self.calculate_scan_coverage(robot_x, robot_y, robot_yaw)
            
            new_coverage = 0
            for point in covered_points:
                if point not in self.scanned_points:
                    self.scanned_points.add(point)
                    new_coverage += 1
            
            coverage_percentage = len(self.scanned_points) / len(self.scan_points) * 100
            
            self.logger.info(f"Scan update: {new_coverage} new points covered, "
                            f"total coverage: {coverage_percentage:.1f}% ({len(self.scanned_points)}/{len(self.scan_points)})")

        def find_optimal_scan_pose(self) -> PoseStamped:
            """
            寻找最优的下一个扫描姿态
            优先选择能覆盖最多未扫描区域的位置
            """
            try:
                if not self.scan_points:
                    self.logger.warn("No scan points available")
                    return None
                    
                # 获取未扫描的点
                unscanned_points = [p for p in self.scan_points if p not in self.scanned_points]
                
                if not unscanned_points:
                    self.logger.info("All points have been scanned!")
                    return None
                    
                self.logger.info(f"Finding optimal pose for {len(unscanned_points)} unscanned points")
                
                best_pose = None
                best_score = -1
                
                # 候选位置生成策略：在未扫描区域周围生成候选位置
                candidate_poses = self._generate_candidate_poses(unscanned_points)
                
                for candidate_x, candidate_y, candidate_yaw in candidate_poses:
                    # 检查位置是否可导航
                    candidate_pose = self.nav_node._create_pose_stamped(candidate_x, candidate_y, candidate_yaw)
                    
                    if not self.nav_node._can_navigate_to_position(candidate_pose):
                        continue
                        
                    # 计算该姿态的覆盖评分
                    score = self._calculate_pose_score(candidate_x, candidate_y, candidate_yaw, unscanned_points)
                    
                    if score > best_score:
                        best_score = score
                        best_pose = candidate_pose
                
                if best_pose:
                    self.logger.info(f"Found optimal scan pose: ({best_pose.pose.position.x:.2f}, {best_pose.pose.position.y:.2f}), score: {best_score:.2f}")
                else:
                    self.logger.warn("No valid scan pose found")
                    
                return best_pose
                
            except Exception as e:
                self.logger.error(f"Error finding optimal scan pose: {e}")
                return None

        def _generate_candidate_poses(self, unscanned_points: list) -> list:
            """
            基于未扫描点生成候选扫描姿态
            
            Returns:
                候选姿态列表 [(x, y, yaw), ...]
            """
            candidates = []
            
            try:
                # 计算未扫描区域的中心
                if not unscanned_points:
                    return candidates
                    
                center_x = sum(p[0] for p in unscanned_points) / len(unscanned_points)
                center_y = sum(p[1] for p in unscanned_points) / len(unscanned_points)
                
                # 在中心周围生成候选位置
                radii = [2.0, 4.0, 6.0, 8.0]  # 不同距离
                angles = [i * math.pi / 8 for i in range(16)]  # 16个方向
                
                for radius in radii:
                    for angle in angles:
                        candidate_x = center_x + radius * math.cos(angle)
                        candidate_y = center_y + radius * math.sin(angle)
                        
                        # 检查候选位置是否在可通行区域
                        if self._is_point_scannable(candidate_x, candidate_y):
                            # 计算面向未扫描区域中心的朝向
                            yaw = math.atan2(center_y - candidate_y, center_x - candidate_x)
                            candidates.append((candidate_x, candidate_y, yaw))
                            
                            # 也添加其他几个朝向
                            for yaw_offset in [-math.pi/3, math.pi/3]:
                                candidates.append((candidate_x, candidate_y, yaw + yaw_offset))
                
                # 随机化顺序避免总是选择相同模式
                import random
                random.shuffle(candidates)
                
                self.logger.debug(f"Generated {len(candidates)} candidate poses")
                return candidates
                
            except Exception as e:
                self.logger.error(f"Error generating candidate poses: {e}")
                return []

        def _calculate_pose_score(self, x: float, y: float, yaw: float, unscanned_points: list) -> float:
            """
            计算扫描姿态的评分
            
            Args:
                x, y, yaw: 候选姿态
                unscanned_points: 未扫描点列表
                
            Returns:
                姿态评分，越高越好
            """
            try:
                # 计算该姿态能覆盖多少未扫描点
                covered_unscanned = 0
                total_coverage = self.calculate_scan_coverage(x, y, yaw)
                
                for point in total_coverage:
                    if point in unscanned_points:
                        covered_unscanned += 1
                
                if covered_unscanned == 0:
                    return 0.0
                    
                # 基础评分：覆盖的未扫描点数量
                coverage_score = covered_unscanned
                
                # 位置优先级评分：距离未扫描区域中心的距离
                if unscanned_points:
                    center_x = sum(p[0] for p in unscanned_points) / len(unscanned_points)
                    center_y = sum(p[1] for p in unscanned_points) / len(unscanned_points)
                    distance_to_center = math.sqrt((x - center_x)**2 + (y - center_y)**2)
                    
                    # 距离适中的位置得分更高（不要太远也不要太近）
                    optimal_distance = self.camera_range * 0.6
                    distance_score = max(0, 1.0 - abs(distance_to_center - optimal_distance) / optimal_distance)
                else:
                    distance_score = 0.5
                
                # 综合评分
                total_score = coverage_score * 10 + distance_score * 2
                
                self.logger.debug(f"Pose ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°): "
                                f"covers {covered_unscanned} points, distance_score={distance_score:.2f}, total={total_score:.2f}")
                
                return total_score
                
            except Exception as e:
                self.logger.error(f"Error calculating pose score: {e}")
                return 0.0

        def get_scan_statistics(self) -> dict:
            """获取扫描统计信息"""
            total_points = len(self.scan_points)
            scanned_points = len(self.scanned_points)
            coverage_percentage = (scanned_points / total_points * 100) if total_points > 0 else 0
            
            return {
                'total_points': total_points,
                'scanned_points': scanned_points,
                'unscanned_points': total_points - scanned_points,
                'coverage_percentage': coverage_percentage
            }

        def visualize_coverage(self) -> str:
            """生成覆盖状态的文本可视化"""
            stats = self.get_scan_statistics()
            
            result = [
                f"=== Region Scan Coverage ===",
                f"Total points: {stats['total_points']}",
                f"Scanned points: {stats['scanned_points']}",
                f"Unscanned points: {stats['unscanned_points']}",
                f"Coverage: {stats['coverage_percentage']:.1f}%",
            ]
            
            # 如果有未扫描点，显示一些示例
            unscanned_points = [p for p in self.scan_points if p not in self.scanned_points]
            if unscanned_points and len(unscanned_points) <= 10:
                result.append("Unscanned points:")
                for i, (x, y) in enumerate(unscanned_points[:5]):
                    result.append(f"  {i+1}: ({x:.2f}, {y:.2f})")
                if len(unscanned_points) > 5:
                    result.append(f"  ... and {len(unscanned_points) - 5} more")
            
            return "\n".join(result)
self.region_scanner = self.RegionScanner(self)


    def execute_region_scan(self, scan_bounds=None) -> bool:
        """
        执行区域扫描
        
        Args:
            scan_bounds: (min_x, min_y, max_x, max_y) 扫描区域，None表示扫描整个地图
            
        Returns:
            扫描是否成功完成
        """
        try:
            self.logger.info("Starting region scan...")
            self.scan_active = True
            
            # 初始化覆盖地图
            if not self.region_scanner.initialize_coverage_map(scan_bounds):
                self.logger.error("Failed to initialize coverage map")
                return False
            
            # 获取当前机器人位置并更新初始覆盖
            robot_pose = self.get_robot_pose_in_map()
            if robot_pose:
                robot_yaw = self.get_robot_yaw_in_map()
                if robot_yaw is not None:
                    self.region_scanner.update_scan_coverage(
                        robot_pose.pose.position.x,
                        robot_pose.pose.position.y,
                        robot_yaw
                    )
            
            # 扫描循环
            max_scan_poses = 50  # 最大扫描姿态数量
            scan_count = 0
            
            while scan_count < max_scan_poses and self.scan_active:
                # 寻找下一个最优扫描位置
                next_pose = self.region_scanner.find_optimal_scan_pose()
                
                if next_pose is None:
                    self.logger.info("Region scan completed - no more optimal poses found")
                    break
                
                # 导航到扫描位置
                self.logger.info(f"Navigating to scan pose {scan_count + 1}: "
                            f"({next_pose.pose.position.x:.2f}, {next_pose.pose.position.y:.2f})")
                
                nav_future = self.nav2pose_async(next_pose)
                rclpy.spin_until_future_complete(self, nav_future)
                
                # 检查导航结果
                try:
                    goal_result = nav_future.result()
                    if not goal_result.accepted:
                        self.logger.warn(f"Navigation to scan pose {scan_count + 1} was rejected")
                        scan_count += 1
                        continue
                        
                    result_future = goal_result.get_result_async()
                    rclpy.spin_until_future_complete(self, result_future)
                    
                    status = result_future.result().status
                    if status != GoalStatus.STATUS_SUCCEEDED:
                        self.logger.warn(f"Navigation to scan pose {scan_count + 1} failed with status: {status}")
                        scan_count += 1
                        continue
                        
                except Exception as e:
                    self.logger.error(f"Error during navigation to scan pose {scan_count + 1}: {e}")
                    scan_count += 1
                    continue
                
                # 到达扫描位置，更新覆盖
                robot_pose = self.get_robot_pose_in_map()
                robot_yaw = self.get_robot_yaw_in_map()
                
                if robot_pose and robot_yaw is not None:
                    self.region_scanner.update_scan_coverage(
                        robot_pose.pose.position.x,
                        robot_pose.pose.position.y,
                        robot_yaw
                    )
                    
                    # 输出当前扫描统计
                    stats = self.region_scanner.get_scan_statistics()
                    self.logger.info(f"Scan pose {scan_count + 1} completed. Coverage: {stats['coverage_percentage']:.1f}%")
                    
                    # 如果覆盖率达到目标，结束扫描
                    if stats['coverage_percentage'] >= 95.0:
                        self.logger.info("Target coverage achieved (95%)")
                        break
                
                scan_count += 1
            
            # 扫描完成
            final_stats = self.region_scanner.get_scan_statistics()
            self.logger.info(f"Region scan completed after {scan_count} poses. "
                            f"Final coverage: {final_stats['coverage_percentage']:.1f}%")
            
            self.scan_active = False
            return True
            
        except Exception as e:
            self.logger.error(f"Error during region scan: {e}")
            self.scan_active = False
            return False

    def stop_region_scan(self):
        """停止区域扫描"""
        self.scan_active = False
        self.navigator.cancelTask()
        self.logger.info("Region scan stopped")
    def execute_region_scan_visualization_only(self, scan_bounds=None) -> bool:
        """
        执行区域扫描的可视化测试 - 仅显示路径和扫描点，不进行实际导航
        
        Args:
            scan_bounds: (min_x, min_y, max_x, max_y) 扫描区域，None表示扫描整个地图
            
        Returns:
            可视化是否成功完成
        """
        try:
            self.logger.info("Starting region scan visualization (no actual navigation)...")
            
            # 初始化覆盖地图
            if not self.region_scanner.initialize_coverage_map(scan_bounds):
                self.logger.error("Failed to initialize coverage map")
                return False
            
            # 获取当前机器人位置
            robot_pose = self.get_robot_pose_in_map()
            if not robot_pose:
                self.logger.error("Cannot get robot pose for visualization")
                return False
                
            robot_yaw = self.get_robot_yaw_in_map()
            if robot_yaw is None:
                self.logger.error("Cannot get robot yaw for visualization")
                return False
            
            # 更新初始覆盖
            self.region_scanner.update_scan_coverage(
                robot_pose.pose.position.x,
                robot_pose.pose.position.y,
                robot_yaw
            )
            
            # 发布初始位置的TF
            self._publish_scan_pose_tf(robot_pose, 0, "current_position")
            
            # 可视化循环
            max_scan_poses = 20  # 减少测试用的姿态数量
            scan_count = 0
            all_scan_poses = []
            all_paths = []
            
            current_pose = robot_pose
            
            while scan_count < max_scan_poses:
                # 寻找下一个最优扫描位置
                next_pose = self.region_scanner.find_optimal_scan_pose()
                
                if next_pose is None:
                    self.logger.info("Region scan visualization completed - no more optimal poses found")
                    break
                
                self.logger.info(f"Scan pose {scan_count + 1}: "
                                f"({next_pose.pose.position.x:.2f}, {next_pose.pose.position.y:.2f})")
                
                # 计算路径（仅用于可视化）
                try:
                    path = self.navigator.getPath(start=current_pose, goal=next_pose)
                    if path and len(path.poses) > 1:
                        path_length = self._calculate_path_length(path)
                        self.logger.info(f"  Path length: {path_length:.2f}m ({len(path.poses)} waypoints)")
                        all_paths.append((scan_count + 1, path))
                    else:
                        self.logger.warn(f"  No valid path found to scan pose {scan_count + 1}")
                        
                except Exception as e:
                    self.logger.warn(f"  Error calculating path to scan pose {scan_count + 1}: {e}")
                
                # 发布扫描位置的TF用于可视化
                self._publish_scan_pose_tf(next_pose, scan_count + 1, f"scan_pose_{scan_count + 1}")
                
                # 模拟到达该位置并更新覆盖（用于算法测试）
                robot_yaw_at_pose = self._extract_yaw_from_pose(next_pose)
                self.region_scanner.update_scan_coverage(
                    next_pose.pose.position.x,
                    next_pose.pose.position.y,
                    robot_yaw_at_pose
                )
                
                # 输出当前扫描统计
                stats = self.region_scanner.get_scan_statistics()
                self.logger.info(f"  Simulated coverage after pose {scan_count + 1}: {stats['coverage_percentage']:.1f}%")
                
                # 保存扫描姿态
                all_scan_poses.append((scan_count + 1, next_pose))
                current_pose = next_pose  # 更新当前位置用于下次路径规划
                
                # 如果覆盖率达到目标，结束扫描
                if stats['coverage_percentage'] >= 95.0:
                    self.logger.info("Target coverage achieved (95%) in simulation")
                    break
                    
                scan_count += 1
                
                # 添加小延迟以便观察
                time.sleep(0.5)
            
            # 输出完整的可视化总结
            self._print_scan_visualization_summary(all_scan_poses, all_paths)
            
            # 保持TF发布一段时间供观察
            self.logger.info("Maintaining TF transforms for visualization (30 seconds)...")
            end_time = time.time() + 30.0
            
            while time.time() < end_time:
                # 重新发布所有TF
                self._publish_scan_pose_tf(robot_pose, 0, "current_position")
                for pose_id, pose in all_scan_poses:
                    self._publish_scan_pose_tf(pose, pose_id, f"scan_pose_{pose_id}")
                time.sleep(1.0)
            
            final_stats = self.region_scanner.get_scan_statistics()
            self.logger.info(f"Region scan visualization completed. "
                            f"Simulated final coverage: {final_stats['coverage_percentage']:.1f}%")
            
            return True
            
        except Exception as e:
            self.logger.error(f"Error during region scan visualization: {e}")
            return False

    def _print_scan_visualization_summary(self, scan_poses: list, paths: list):
        """打印扫描可视化总结"""
        self.logger.info("=== SCAN VISUALIZATION SUMMARY ===")
        self.logger.info(f"Total scan poses planned: {len(scan_poses)}")
        
        if scan_poses:
            self.logger.info("Scan poses:")
            for pose_id, pose in scan_poses:
                yaw_deg = math.degrees(self._extract_yaw_from_pose(pose))
                self.logger.info(f"  Pose {pose_id}: ({pose.pose.position.x:.2f}, "
                                f"{pose.pose.position.y:.2f}, {yaw_deg:.1f}°)")
        
        if paths:
            total_path_length = 0.0
            self.logger.info("Path information:")
            for pose_id, path in paths:
                length = self._calculate_path_length(path)
                total_path_length += length
                self.logger.info(f"  To Pose {pose_id}: {length:.2f}m ({len(path.poses)} waypoints)")
            
            self.logger.info(f"Total planned path length: {total_path_length:.2f}m")
        
        # 输出覆盖信息
        stats = self.region_scanner.get_scan_statistics()
        self.logger.info(f"Simulated coverage: {stats['coverage_percentage']:.1f}%")
        self.logger.info("=== END SUMMARY ===")


    # 添加socket命令
    def region_scan_viz_cmd(self, *args) -> str:
        """区域扫描可视化命令"""
        if len(args) == 0:
            # 可视化整个区域的扫描
            success = self.execute_region_scan_visualization_only()
            return f"Region scan visualization {'completed' if success else 'failed'}"
        elif len(args) == 4:
            # 可视化指定区域的扫描
            try:
                bounds = (float(args[0]), float(args[1]), float(args[2]), float(args[3]))
                success = self.execute_region_scan_visualization_only(bounds)
                return f"Region scan visualization {'completed' if success else 'failed'}"
            except ValueError:
                return "ERROR: Invalid bounds format. Use: region_scan_viz_cmd min_x min_y max_x max_y"
        else:
            return "Usage: region_scan_viz_cmd [min_x min_y max_x max_y]"
        
    # Socket 命令
    def region_scan_cmd(self, *args) -> str:
        """区域扫描命令"""
        if len(args) == 0:
            # 扫描整个区域
            success = self.execute_region_scan()
            return f"Region scan {'completed' if success else 'failed'}"
        elif len(args) == 4:
            # 扫描指定区域
            try:
                bounds = (float(args[0]), float(args[1]), float(args[2]), float(args[3]))
                success = self.execute_region_scan(bounds)
                return f"Region scan {'completed' if success else 'failed'}"
            except ValueError:
                return "ERROR: Invalid bounds format. Use: region_scan_cmd min_x min_y max_x max_y"
        else:
            return "Usage: region_scan_cmd [min_x min_y max_x max_y]"

    def scan_status_cmd(self) -> str:
        """获取扫描状态"""
        if hasattr(self, 'region_scanner') and self.region_scanner.scan_points:
            return self.region_scanner.visualize_coverage()
        else:
            return "No active scan session"

    def stop_scan_cmd(self) -> str:
        """停止扫描命令"""
        self.stop_region_scan()
        return "Region scan stopped"
