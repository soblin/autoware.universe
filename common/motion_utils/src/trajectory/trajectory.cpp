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
template boost::optional<size_t> findNearestIndex(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Pose & pose, const double max_dist, const double max_yaw);
template double calcLongitudinalOffsetToSegment(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const size_t seg_idx, const geometry_msgs::msg::Point & p_target, const bool throw_exception);
template size_t findNearestSegmentIndex(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & point);
template boost::optional<size_t> findNearestSegmentIndex(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Pose & pose, const double max_dist, const double max_yaw);
template double calcLateralOffset(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & p_target, const size_t seg_idx, const bool throw_exception);
template double calcLateralOffset(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & p_target, const bool throw_exception);
template double calcSignedArcLength(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const size_t src_idx, const size_t dst_idx);
template double calcSignedArcLength(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & src_point, const size_t dst_idx);
template boost::optional<double> calcSignedArcLength(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Pose & src_pose, const size_t dst_idx, const double max_dist,
  const double max_yaw);
template double calcSignedArcLength(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const size_t src_idx, const geometry_msgs::msg::Point & dst_point);
template double calcSignedArcLength(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & src_point, const geometry_msgs::msg::Point & dst_point);
template boost::optional<double> calcSignedArcLength(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Pose & src_pose, const geometry_msgs::msg::Point & dst_point,
  const double max_dist, const double max_yaw);
template double calcArcLength(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points);
template std::vector<double> calcCurvature(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points);
template std::vector<std::pair<double, double>> calcCurvatureAndArcLength(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points);
template boost::optional<double> calcDistanceToForwardStopPoint(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points_with_twist,
  const size_t);
template boost::optional<double> calcDistanceToForwardStopPoint(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points_with_twist,
  const geometry_msgs::msg::Pose & pose, const double max_dist, const double max_yaw);
template boost::optional<geometry_msgs::msg::Point> calcLongitudinalOffsetPoint(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const size_t src_idx, const double offset, const bool throw_exception);
template boost::optional<geometry_msgs::msg::Point> calcLongitudinalOffsetPoint(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & src_point, const double offset);
template boost::optional<geometry_msgs::msg::Pose> calcLongitudinalOffsetPose(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const size_t src_idx, const double offset, const bool set_orientation_from_position_direction,
  const bool throw_exception);
template boost::optional<geometry_msgs::msg::Pose> calcLongitudinalOffsetPose(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & src_point, const double offset,
  const bool set_orientation_from_position_direction);
template boost::optional<size_t> insertTargetPoint(
  const size_t seg_idx, const geometry_msgs::msg::Point & p_target,
  std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const double overlap_threshold);
template boost::optional<size_t> insertTargetPoint(
  const double insert_point_length, const geometry_msgs::msg::Point & p_target,
  std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const double overlap_threshold);
template boost::optional<size_t> insertTargetPoint(
  const size_t src_segment_idx, const double insert_point_length,
  std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const double overlap_threshold);
template boost::optional<size_t> insertTargetPoint(
  const geometry_msgs::msg::Pose & src_pose, const double insert_point_length,
  std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const double max_dist, const double max_yaw, const double overlap_threshold);
template boost::optional<size_t> insertStopPoint(
  const size_t src_segment_idx, const double distance_to_stop_point,
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points_with_twist,
  const double overlap_threshold);
template boost::optional<size_t> insertStopPoint(
  const geometry_msgs::msg::Pose & src_pose, const double distance_to_stop_point,
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points_with_twist,
  const double max_dist, const double max_yaw, const double overlap_threshold);
template void insertOrientation(
  std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const bool is_driving_forward);
template double calcSignedArcLength(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & src_point, const size_t src_seg_idx,
  const geometry_msgs::msg::Point & dst_point, const size_t dst_seg_idx);
template double calcSignedArcLength(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & src_point, const size_t src_seg_idx, const size_t dst_idx);
template double calcSignedArcLength(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const size_t src_idx, const geometry_msgs::msg::Point & dst_point, const size_t dst_seg_idx);
template size_t findFirstNearestIndexWithSoftConstraints(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Pose & pose, const double dist_threshold, const double yaw_threshold);
template size_t findFirstNearestSegmentIndexWithSoftConstraints(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Pose & pose, const double dist_threshold, const double yaw_threshold);
}  // namespace motion_utils
