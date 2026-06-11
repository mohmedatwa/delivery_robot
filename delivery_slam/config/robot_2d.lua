include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "base_footprint",
  odom_frame = "odom",

  -- false: EKF (robot_localization) already owns the odom→base_footprint TF.
  -- true would create a duplicate TF broadcaster that corrupts Nav2's TF tree.
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,

  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,

  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  lookup_transform_timeout_sec = 0.2,

  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.005,
  trajectory_publish_period_sec = 0.03,

  -- 0.33: throttles ~15 Hz sim scan input down to ~5 Hz inside Cartographer.
  -- Fixes the real-time factor drop to 64% caused by the laser plugin
  -- publishing at 15 Hz instead of the expected 5 Hz.
  -- If you fix update_rate in the URDF/SDF to 5.0, set this back to 1.0.
  rangefinder_sampling_ratio = 0.33,

  -- 0.5: accept every other odom message.
  -- Fixes the "data.time > prev->first" SIGABRT crash caused by
  -- /mecanum_controller/odom publishing duplicate timestamps when
  -- the controller rate exceeds system clock resolution.
  odometry_sampling_ratio = 0.5,

  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- RPLIDAR A1 spec: 0.15–12 m. 0.10 m floor gives a small safety margin.
TRAJECTORY_BUILDER_2D.min_range = 0.10
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0

-- IMU data is consumed by the EKF, not fed directly to Cartographer.
TRAJECTORY_BUILDER_2D.use_imu_data = false

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- Motion filter: how much the robot must move/rotate before a new node
-- is inserted. Cartographer defaults (5 s / 0.2 m / 1.0 deg) are too
-- coarse for slow indoor delivery maneuvers — only 7.6% of scans were
-- kept with defaults. These tighter thresholds produce a denser, more
-- accurate map in corridor/room-scale environments.
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)

-- Tuned for small indoor environment: more frequent loop closure
-- at slightly higher CPU cost. Raise back to 90 for large spaces.
POSE_GRAPH.optimize_every_n_nodes = 30

return options