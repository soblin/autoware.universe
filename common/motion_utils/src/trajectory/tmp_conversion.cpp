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

#include <motion_utils/trajectory/tmp_conversion.hpp>

namespace motion_utils
{
autoware_auto_planning_msgs::msg::Trajectory convertToTrajectory(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & trajectory)
{
  autoware_auto_planning_msgs::msg::Trajectory output{};
  for (const auto & pt : trajectory) {
    output.points.push_back(pt);
    if (output.points.size() >= output.CAPACITY) {
      break;
    }
  }
  return output;
}

/**
 * @brief Convert autoware_auto_planning_msgs::msg::Trajectory to
 * std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>.
 */
std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> convertToTrajectoryPointArray(
  const autoware_auto_planning_msgs::msg::Trajectory & trajectory)
{
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> output(trajectory.points.size());
  std::copy(trajectory.points.begin(), trajectory.points.end(), output.begin());
  return output;
}

}  // namespace motion_utils
