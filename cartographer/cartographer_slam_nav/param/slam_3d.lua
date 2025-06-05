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
  -- true改为false，不用提供里程计数据
  provide_odom_frame = false,
  use_trajectory_builder_3d = true,
  -- false改为true，使用里程计数据
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  -- 0改为1,使用一个雷达
  num_laser_scans = 1,
  -- 1改为0，不使用多波雷达
  num_multi_echo_laser_scans = 0,
  -- 10改为1，1/1=1等于不分割
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}


-- false改为true，启动2D SLAM
MAP_BUILDER.use_trajectory_builder_3d = true

TRAJECTORY_BUILDER_3D.min_range = 0.40
TRAJECTORY_BUILDER_3D.max_range = 5.5

-- 0.55改成0.65,Fast csm的最低分数，高于此分数才进行优化。
POSE_GRAPH.constraint_builder.min_score = 0.65
--0.6改成0.7,全局定位最小分数，低于此分数则认为目前全局定位不准确
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

-- 设置0可关闭全局SLAM
-- POSE_GRAPH.optimize_every_n_nodes = 0

return options
