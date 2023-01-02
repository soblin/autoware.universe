// Copyright 2022 TIER IV, Inc.
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

#ifndef MOTION_UTILS__TRAJECTORY__PATH_WITH_LANE_ID_HPP_
#define MOTION_UTILS__TRAJECTORY__PATH_WITH_LANE_ID_HPP_

#include "motion_utils/trajectory/trajectory.hpp"
#include "tier4_autoware_utils/geometry/path_with_lane_id_geometry.hpp"

#include <boost/optional.hpp>

#include <algorithm>
#include <utility>
#include <vector>

namespace motion_utils
{
boost::optional<std::pair<size_t, size_t>> getPathIndexRangeWithLaneId(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const int64_t target_lane_id);

size_t findNearestIndexFromLaneId(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const geometry_msgs::msg::Point & pos, const int64_t lane_id);

size_t findNearestSegmentIndexFromLaneId(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const geometry_msgs::msg::Point & pos, const int64_t lane_id);
}  // namespace motion_utils

#endif  // MOTION_UTILS__TRAJECTORY__PATH_WITH_LANE_ID_HPP_
