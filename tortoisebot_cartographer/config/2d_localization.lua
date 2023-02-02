-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "2d_slam.lua"
-- cartographer localization optimisation reference: https://google-cartographer-ros.readthedocs.io/en/latest/tuning.html#pure-localization-in-a-given-map

TRAJECTORY_BUILDER.pure_localization = true
-- fast localization
MAP_BUILDER.num_background_threads = 12
POSE_GRAPH.constraint_builder.sampling_ratio = 0.5 * POSE_GRAPH.constraint_builder.sampling_ratio
POSE_GRAPH.global_sampling_ratio = 0.1 * POSE_GRAPH.global_sampling_ratio
POSE_GRAPH.max_num_final_iterations = 1
POSE_GRAPH.optimize_every_n_nodes = 2

-- Following is with reference from /opt/ros/galactic/share/turtlebot3_cartographer/config/turtlebot3_lds_2d.lua
-- options.tracking_frame = "imu_link"
-- options.use_odometry = true
-- options.published_frame = "odom"
-- options.provide_odom_frame = false
-- options.odom_frame = "odom"
-- options.publish_frame_projected_to_2d = true

return options
