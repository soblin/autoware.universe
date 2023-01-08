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

#ifndef UTILIZATION__ARC_LANE_UTIL_HPP_
#define UTILIZATION__ARC_LANE_UTIL_HPP_

#include <utilization/util.hpp>

#include <boost/optional.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <utility>

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;

namespace arc_lane_utils
{
using PathIndexWithPose = std::pair<size_t, geometry_msgs::msg::Pose>;    // front index, pose
using PathIndexWithPoint2d = std::pair<size_t, Point2d>;                  // front index, point2d
using PathIndexWithPoint = std::pair<size_t, geometry_msgs::msg::Point>;  // front index, point2d
using PathIndexWithOffset = std::pair<size_t, double>;                    // front index, offset

double calcSignedDistance(
  const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Point & p2);

// calculate one collision point between the line (from p1 to p2) and the line (from p3 to p4)
boost::optional<geometry_msgs::msg::Point> checkCollision(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2,
  const geometry_msgs::msg::Point & p3, const geometry_msgs::msg::Point & p4);

boost::optional<PathIndexWithPoint> findCollisionSegment(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const geometry_msgs::msg::Point & stop_line_p1, const geometry_msgs::msg::Point & stop_line_p2,
  const size_t target_lane_id);

boost::optional<PathIndexWithPoint> findCollisionSegment(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const LineString2d & stop_line,
  const size_t target_lane_id);

boost::optional<PathIndexWithPose> createTargetPoint(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const LineString2d & stop_line,
  const size_t lane_id, const double margin, const double vehicle_offset);

boost::optional<PathIndexWithOffset> findForwardOffsetSegment(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const size_t base_idx,
  const double offset_length);

boost::optional<PathIndexWithOffset> findBackwardOffsetSegment(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const size_t base_idx,
  const double offset_length);

boost::optional<PathIndexWithOffset> findOffsetSegment(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const PathIndexWithPoint & collision_segment, const double offset_length);

boost::optional<PathIndexWithOffset> findOffsetSegment(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const size_t index,
  const double offset);

geometry_msgs::msg::Pose calcTargetPose(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const PathIndexWithOffset & offset_segment);

boost::optional<PathIndexWithPose> createTargetPoint(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const LineString2d & stop_line,
  const size_t lane_id, const double margin, const double vehicle_offset);
}  // namespace arc_lane_utils
}  // namespace behavior_velocity_planner

#endif  // UTILIZATION__ARC_LANE_UTIL_HPP_
