// Copyright 2022 Tier IV, Inc.
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

#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/geometry/path_with_lane_id_geometry.hpp>

namespace motion_utils
{
template void validateNonEmpty(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> &);
template void validateNonSharpAngle(
  const autoware_auto_planning_msgs::msg::PathPointWithLaneId &,
  const autoware_auto_planning_msgs::msg::PathPointWithLaneId &,
  const autoware_auto_planning_msgs::msg::PathPointWithLaneId &, const double);
template boost::optional<bool> isDrivingForward(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> &);
template boost::optional<bool> isDrivingForwardWithTwist(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> &);
template std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> removeOverlapPoints(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> &, const size_t &);
template boost::optional<size_t> searchZeroVelocityIndex(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> &, const size_t,
  const size_t);
template boost::optional<size_t> searchZeroVelocityIndex(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> &, const size_t &);
template boost::optional<size_t> searchZeroVelocityIndex(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> &);
template size_t findNearestIndex(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> &,
  const geometry_msgs::msg::Point &);

}  // namespace motion_utils
