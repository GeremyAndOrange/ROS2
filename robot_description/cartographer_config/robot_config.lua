include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "submap",                         -- 全局坐标系
  tracking_frame = "base_footprint",            -- 自身坐标系
  published_frame = "odom",                     -- odom到map的tf转换
  odom_frame = "odom",                          -- odom坐标系
  provide_odom_frame = false,                   -- 
  publish_frame_projected_to_2d = true,         -- 发布2D位姿
  use_odometry = true,                          -- 
  use_nav_sat = false,                          -- 不使用GPS数据
  use_landmarks = false,                        -- 不使用地标数据
  num_laser_scans = 1,                          -- 雷达数量
  num_multi_echo_laser_scans = 0,               -- 多波雷达数量
  num_subdivisions_per_laser_scan = 1,          -- 雷达分割数量
  num_point_clouds = 0,                         -- 点云数据数量
  lookup_transform_timeout_sec = 1,             -- 坐标转换超时时间
  submap_publish_period_sec = 0.3,              -- 子图发布周期
  pose_publish_period_sec = 5e-3,               -- 位姿发布周期
  trajectory_publish_period_sec = 30e-3,        -- 轨迹发布周期
  rangefinder_sampling_ratio = 1.,              -- 测距仪采样比率
  odometry_sampling_ratio = 1.,                 -- 里程计采样比率
  fixed_frame_pose_sampling_ratio = 1.,         -- 位姿采样比率
  imu_sampling_ratio = 1.,                      -- IMU采样比率
  landmarks_sampling_ratio = 1.,                -- 地标采样比率
}

MAP_BUILDER.use_trajectory_builder_2d = true    -- 2D SLAM
TRAJECTORY_BUILDER_2D.min_range = 0.10          -- 2D轨迹构建器接受的最小激光雷达扫描距离
TRAJECTORY_BUILDER_2D.max_range = 3.5           -- 2D轨迹构建器接受的最大激光雷达扫描距离
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.                    -- 缺失雷达数据时假定的最大距离
TRAJECTORY_BUILDER_2D.use_imu_data = true       -- 使用IMU数据
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true     -- 使用扫描匹配
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1) -- 运动滤波器最大角度
POSE_GRAPH.constraint_builder.min_score = 0.75  -- 地图回环置信度(约束构建器最小分数)
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.8     -- 全局定位最小分数
POSE_GRAPH.optimize_every_n_nodes = 100         -- 全局优化频率(每n个节点)
POSE_GRAPH.constraint_builder.sampling_ratio = 0.4

return options