// Copyright 2020 Tier IV, Inc.
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

#ifndef SCENE_MODULE__CROSSWALK__UTIL_HPP_
#define SCENE_MODULE__CROSSWALK__UTIL_HPP_

#include "behavior_velocity_planner/planner_data.hpp"
#include "utilization/boost_geometry_helper.hpp"

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace behavior_velocity_planner
{

namespace bg = boost::geometry;
using autoware_auto_planning_msgs::msg::PathPointWithLaneId;
using autoware_auto_planning_msgs::msg::PathWithLaneId;

enum class CollisionPointState { YIELD, EGO_PASS_FIRST, EGO_PASS_LATER, IGNORE };

struct CollisionPoint
{
  geometry_msgs::msg::Point collision_point{};
  double time_to_collision{};
  double time_to_vehicle{};
  CollisionPointState state{CollisionPointState::EGO_PASS_FIRST};
};

struct DebugData
{
  bool ignore_crosswalk{false};
  double base_link2front;
  double stop_judge_range;

  geometry_msgs::msg::Pose first_stop_pose;
  geometry_msgs::msg::Point nearest_collision_point;

  boost::optional<geometry_msgs::msg::Point> range_near_point{boost::none};
  boost::optional<geometry_msgs::msg::Point> range_far_point{boost::none};

  std::vector<CollisionPoint> collision_points;

  std::vector<geometry_msgs::msg::Pose> stop_poses;
  std::vector<geometry_msgs::msg::Pose> slow_poses;
  std::vector<geometry_msgs::msg::Point> stop_factor_points;
  std::vector<geometry_msgs::msg::Point> crosswalk_polygon;
  std::vector<geometry_msgs::msg::Polygon> ego_polygons;
  std::vector<geometry_msgs::msg::Polygon> obj_polygons;
};

std::vector<Point2d> getPolygonIntersects(
  const PathWithLaneId & ego_path, const lanelet::BasicPolygon2d & polygon,
  const geometry_msgs::msg::Point & ego_pos, const size_t max_num);

std::vector<Point2d> getLinestringIntersects(
  const PathWithLaneId & ego_path, const lanelet::BasicLineString2d & linestring,
  const geometry_msgs::msg::Point & ego_pos, const size_t max_num);

lanelet::Optional<lanelet::ConstLineString3d> getStopLineFromMap(
  const int lane_id, const std::shared_ptr<const PlannerData> & planner_data,
  const std::string & attribute_name);
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__CROSSWALK__UTIL_HPP_
