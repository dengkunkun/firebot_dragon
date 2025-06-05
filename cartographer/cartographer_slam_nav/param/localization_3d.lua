-- Pure localization configuration

include "slam_3d.lua"

-- 启用纯定位模式
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

POSE_GRAPH.optimize_every_n_nodes = 20

return options