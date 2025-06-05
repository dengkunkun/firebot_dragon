-- 纯定位配置，基于slam_2d.lua优化

include "slam_2d.lua"


-- -- ===== Frame配置调整 =====
-- -- 定位模式下的frame配置与SLAM不同
-- options.tracking_frame = "imu_link"  -- 改为imu_link，直接使用IMU数据
-- options.published_frame = "base_link"    -- 改为base_link，直接发布机器人位姿
-- options.provide_odom_frame = true       -- 保持false，使用机器人自己的odom
-- options.use_odometry = false              -- 保持true，利用高质量odom
-- options.odom_frame = "odom_cartographer"  -- 保持odom_cartographer
-- options.imu_sampling_ratio = 1.0  -- 设为0禁用IMU采样
-- -- ===== 定位优化参数 =====

-- -- 更频繁的位姿图优化（原来50改为20）
-- POSE_GRAPH.optimize_every_n_nodes = 20

-- -- 定位模式下的扫描匹配优化
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.2  -- 增大搜索窗口
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(30.)  -- 增大角度搜索

-- -- 定位模式下的Ceres优化参数
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 15  -- 增加平移权重
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 20     -- 增加旋转权重

-- -- 约束构建器优化（定位模式需要更敏感的约束检测）
-- POSE_GRAPH.constraint_builder.min_score = 0.65                    -- 降低最小分数，更容易建立约束
-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.5                -- 增加采样率
-- POSE_GRAPH.global_sampling_ratio = 0.005                          -- 增加全局采样率

-- -- 全局定位参数优化
-- POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65  -- 降低全局定位门槛

-- -- 定位模式下的快速相关扫描匹配器
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 4.  -- 增大搜索窗口
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(20.)

-- -- 减少计算负载（定位比建图计算量小）
-- TRAJECTORY_BUILDER_2D.submaps.num_range_data = 80               -- 减少到80（原来120）

-- -- 定位模式下的运动过滤器（更敏感）
-- TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.15    -- 稍微减小
-- TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.8)  -- 稍微减小

-- -- ===== 定位专用的IMU使用策略 =====
-- -- 定位模式下建议开启IMU以提高精度
-- TRAJECTORY_BUILDER_2D.use_imu_data = true   -- 改为true，定位时IMU很有用

-- -- ===== 定位响应频率优化 =====
-- options.pose_publish_period_sec = 0.01      -- 增加位姿发布频率到100Hz
-- options.trajectory_publish_period_sec = 20e-3  -- 稍微增加轨迹发布频率


TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 5,
}

return options