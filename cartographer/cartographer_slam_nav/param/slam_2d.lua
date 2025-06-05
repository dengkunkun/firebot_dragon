include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  -- base_link改为odom,发布map到odom之间的位姿态
  published_frame = "odom",
  odom_frame = "odom",
  -- 是否会发布从 odom_frame 到 tracking_frame 的 TF 变换，如果不自带里程计，可以设为true
  provide_odom_frame = false,
  -- false改为true，仅发布2D位资
  publish_frame_projected_to_2d = true,
  -- false改为true，使用里程计数据
  use_odometry = true,
  -- gps
  use_nav_sat = false,
  use_landmarks = false,
  -- 0改为1,使用一个雷达
  num_laser_scans = 1,
  -- 1改为0，不使用多波雷达
  num_multi_echo_laser_scans = 0,
  -- 10改为1，1/1=1等于不分割
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.5,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.02,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}


-- false改为true，启动2D SLAM
MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
-- 0改成0.10,比机器人半径小的都忽略
TRAJECTORY_BUILDER_2D.min_range = 0.4
-- 30改成3.5,限制在雷达最大扫描范围内，越小一般越精确些
-- TRAJECTORY_BUILDER_2D.max_range = 5.5
-- 5改成3,传感器数据超出有效范围最大值
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 4.
-- true改成false,不使用IMU数据，大家可以开启，然后对比下效果
TRAJECTORY_BUILDER_2D.use_imu_data = false
-- false改成true,使用实时回环检测来进行前端的扫描匹配
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true 
-- 1.0改成0.1,提高对运动的敏感度
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.2)
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.1
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 15

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 15
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.58

-- 0.55改成0.65,Fast csm的最低分数，高于此分数才进行优化。
POSE_GRAPH.constraint_builder.min_score = 0.55
--0.6改成0.7,全局定位最小分数，低于此分数则认为目前全局定位不准确
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(15.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 3.
POSE_GRAPH.optimization_problem.huber_scale = 1e2
-----------------TUNE THESE PARAMETERS FOR LOW LATENCY-------------------------------

------------Global SLAM------------
POSE_GRAPH.optimize_every_n_nodes = 50 -- Decrease
POSE_GRAPH.global_sampling_ratio = 0.003 -- Decrease
POSE_GRAPH.constraint_builder.sampling_ratio = 0.4 -- Decrease
POSE_GRAPH.global_constraint_search_after_n_seconds = 15 -- Increase
POSE_GRAPH.constraint_builder.max_constraint_distance = 18

---------Global/Local SLAM---------
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90 -- Decrease
TRAJECTORY_BUILDER_2D.max_range = 5.5 -- Decrease

-- 设置0可关闭全局SLAM
-- POSE_GRAPH.optimize_every_n_nodes = 0

return options
