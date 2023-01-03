// Copyright 2021 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef UTILIZATION__TRAJECTORY_UTILS_HPP_
#define UTILIZATION__TRAJECTORY_UTILS_HPP_

#include <behavior_velocity_planner/planner_data.hpp>
#include <interpolation/linear_interpolation.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <motion_velocity_smoother/trajectory_utils.hpp>

#include <autoware_auto_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
using autoware_auto_planning_msgs::msg::PathPointWithLaneId;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using geometry_msgs::msg::Quaternion;
using TrajectoryPointWithIdx = std::pair<TrajectoryPoint, size_t>;

TrajectoryPoints convertPathToTrajectoryPoints(const PathWithLaneId & path);
PathWithLaneId convertTrajectoryPointsToPath(const TrajectoryPoints & trajectory);

Quaternion lerpOrientation(const Quaternion & o_from, const Quaternion & o_to, const double ratio);

//! smooth path point with lane id starts from ego position on path to the path end
bool smoothPath(
  const PathWithLaneId & in_path, PathWithLaneId & out_path,
  const std::shared_ptr<const PlannerData> & planner_data);

}  // namespace behavior_velocity_planner

#endif  // UTILIZATION__TRAJECTORY_UTILS_HPP_
