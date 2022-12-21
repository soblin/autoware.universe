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

#include <lanelet2_extension/regulatory_elements/road_marking.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <opencv2/imgproc.hpp>
#include <scene_module/intersection/scene_intersection.hpp>
#include <scene_module/intersection/util.hpp>
#include <utilization/boost_geometry_helper.hpp>
#include <utilization/path_utilization.hpp>
#include <utilization/trajectory_utils.hpp>
#include <utilization/util.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <tf2/convert.h>
#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;

static geometry_msgs::msg::Pose getObjectPoseWithVelocityDirection(
  const autoware_auto_perception_msgs::msg::PredictedObjectKinematics & obj_state)
{
  if (obj_state.initial_twist_with_covariance.twist.linear.x >= 0) {
    return obj_state.initial_pose_with_covariance.pose;
  }

  // When the object velocity is negative, invert orientation (yaw)
  auto obj_pose = obj_state.initial_pose_with_covariance.pose;
  double yaw, pitch, roll;
  tf2::getEulerYPR(obj_pose.orientation, yaw, pitch, roll);
  tf2::Quaternion inv_q;
  inv_q.setRPY(roll, pitch, yaw + M_PI);
  obj_pose.orientation = tf2::toMsg(inv_q);
  return obj_pose;
}

IntersectionModule::IntersectionModule(
  const int64_t module_id, const int64_t lane_id, std::shared_ptr<const PlannerData> planner_data,
  const PlannerParam & planner_param, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock), lane_id_(lane_id), is_go_out_(false)
{
  planner_param_ = planner_param;
  const auto & assigned_lanelet =
    planner_data->route_handler_->getLaneletMapPtr()->laneletLayer.get(lane_id);
  turn_direction_ = assigned_lanelet.attributeOr("turn_direction", "else");
  has_traffic_light_ =
    !(assigned_lanelet.regulatoryElementsAs<const lanelet::TrafficLight>().empty());
  state_machine_.setMarginTime(planner_param_.state_transit_margin_time);
}

bool IntersectionModule::modifyPathVelocity(
  autoware_auto_planning_msgs::msg::PathWithLaneId * path,
  tier4_planning_msgs::msg::StopReason * stop_reason)
{
  RCLCPP_DEBUG(logger_, "===== plan start =====");
  const bool external_go = isTargetExternalInputStatus(tier4_api_msgs::msg::IntersectionStatus::GO);
  const bool external_stop =
    isTargetExternalInputStatus(tier4_api_msgs::msg::IntersectionStatus::STOP);
  const State current_state = state_machine_.getState();

  debug_data_ = DebugData();
  debug_data_.path_raw = *path;

  *stop_reason =
    planning_utils::initializeStopReason(tier4_planning_msgs::msg::StopReason::INTERSECTION);

  RCLCPP_DEBUG(
    logger_, "lane_id = %ld, state = %s", lane_id_,
    IntersectionModule::toString(current_state).c_str());

  /* get current pose */
  const geometry_msgs::msg::PoseStamped current_pose = planner_data_->current_pose;
  const double current_vel = planner_data_->current_velocity->twist.linear.x;

  /* get lanelet map */
  const auto lanelet_map_ptr = planner_data_->route_handler_->getLaneletMapPtr();
  const auto routing_graph_ptr = planner_data_->route_handler_->getRoutingGraphPtr();
  const auto & assigned_lanelet =
    planner_data_->route_handler_->getLaneletMapPtr()->laneletLayer.get(lane_id_);
  const std::string turn_direction = assigned_lanelet.attributeOr("turn_direction", "else");

  /* get detection area*/
  /* dynamically change detection area based on tl_arrow_solid_on */
  const bool tl_arrow_solid_on =
    util::isTrafficLightArrowActivated(assigned_lanelet, planner_data_->traffic_light_id_map);
  if (!intersection_lanelets_.has_value() || intersection_lanelets_.value().tl_arrow_solid_on) {
    intersection_lanelets_ = util::getObjectiveLanelets(
      lanelet_map_ptr, routing_graph_ptr, lane_id_, planner_param_.detection_area_length,
      planner_data_->occupancy_grid->info.width * std::sqrt(2.0), tl_arrow_solid_on);
  }
  const auto & detection_lanelets = intersection_lanelets_.value().attention;
  const auto & adjacent_lanelets = intersection_lanelets_.value().adjacent;
  const auto & aux_detection_lanelets = intersection_lanelets_.value().aux_attention;
  const auto & detection_area = intersection_lanelets_.value().attention_area;
  const auto & conflicting_area = intersection_lanelets_.value().conflicting_area;
  const auto & aux_attention_area = intersection_lanelets_.value().aux_attention_area;
  debug_data_.detection_area = detection_area;

  /* get intersection area */
  const auto intersection_area = util::getIntersectionArea(assigned_lanelet, lanelet_map_ptr);
  if (intersection_area) {
    const auto intersection_area_2d = intersection_area.value();
    debug_data_.intersection_area = toGeomMsg(intersection_area_2d);
  }

  constexpr double interval = 0.2;  // NOTE should be smaller than occ_grid resolution
  autoware_auto_planning_msgs::msg::PathWithLaneId path_ip;
  if (!splineInterpolate(*path, interval, path_ip, logger_)) {
    RCLCPP_DEBUG_SKIPFIRST_THROTTLE(logger_, *clock_, 1000 /* ms */, "interpolation failed");
    setSafe(true);
    setDistance(std::numeric_limits<double>::lowest());
    return false;
  }

  if (!detection_divisions_.has_value()) {
    detection_divisions_ = util::generateDetectionLaneDivisions(
      aux_detection_lanelets, routing_graph_ptr,
      planner_data_->occupancy_grid->info.resolution / std::sqrt(2.0));
  }

  // use interpolated path
  showOccupancyGridImage(
    lane_id_, *planner_data_->occupancy_grid, aux_attention_area, path_ip,
    detection_divisions_.value());

  /* set stop lines for base_link */
  const auto [stuck_line_idx_opt, stop_lines_idx_opt] = util::generateStopLine(
    lane_id_, detection_area, conflicting_area, planner_data_, planner_param_.stop_line_margin,
    planner_param_.keep_detection_line_margin, planner_param_.use_stuck_stopline, path, *path,
    logger_.get_child("util"), clock_);
  if (!stuck_line_idx_opt.has_value()) {
    // returns here if path is not intersecting with conflicting areas
    RCLCPP_DEBUG_SKIPFIRST_THROTTLE(
      logger_, *clock_, 1000 /* ms */, "setStopLineIdx for stuck line fail");
    RCLCPP_DEBUG(logger_, "===== plan end =====");
    return false;
  }
  const auto stuck_line_idx = stuck_line_idx_opt.value();

  /* calc closest index */
  int closest_idx = -1;
  if (!planning_utils::calcClosestIndex(*path, current_pose.pose, closest_idx)) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(logger_, *clock_, 1000 /* ms */, "calcClosestIndex fail");
    RCLCPP_DEBUG(logger_, "===== plan end =====");
    return false;
  }

  if (stop_lines_idx_opt.has_value()) {
    const auto stop_line_idx = stop_lines_idx_opt.value().stop_line;
    const auto pass_judge_line_idx = stop_lines_idx_opt.value().pass_judge_line;
    const auto keep_detection_line_idx = stop_lines_idx_opt.value().keep_detection_line;

    const bool is_over_pass_judge_line =
      util::isOverTargetIndex(*path, closest_idx, current_pose.pose, pass_judge_line_idx);
    const bool is_before_keep_detection_line =
      stop_lines_idx_opt.has_value()
        ? util::isBeforeTargetIndex(*path, closest_idx, current_pose.pose, keep_detection_line_idx)
        : false;
    const bool keep_detection = is_before_keep_detection_line &&
                                std::fabs(current_vel) < planner_param_.keep_detection_vel_thr;

    if (is_over_pass_judge_line && keep_detection) {
      // in case ego could not stop exactly before the stop line, but with some overshoot,
      // keep detection within some margin under low velocity threshold
    } else if (is_over_pass_judge_line && is_go_out_ && !external_stop) {
      RCLCPP_INFO(logger_, "over the keep_detection line and not low speed. no plan needed.");
      RCLCPP_DEBUG(logger_, "===== plan end =====");
      setSafe(true);
      setDistance(motion_utils::calcSignedArcLength(
        path->points, planner_data_->current_pose.pose.position,
        path->points.at(stop_line_idx).point.pose.position));
      return true;
    }
  }
  /* collision checking */
  bool is_entry_prohibited = false;

  /* get dynamic object */
  const auto objects_ptr = planner_data_->predicted_objects;

  /* check stuck vehicle */
  const double ignore_length =
    planner_param_.stuck_vehicle_ignore_dist + planner_data_->vehicle_info_.vehicle_length_m;
  const double detect_dist =
    planner_param_.stuck_vehicle_detect_dist + planner_data_->vehicle_info_.vehicle_length_m;
  const auto stuck_vehicle_detect_area = generateEgoIntersectionLanePolygon(
    lanelet_map_ptr, *path, closest_idx, detect_dist, ignore_length);
  const bool is_stuck = checkStuckVehicleInIntersection(objects_ptr, stuck_vehicle_detect_area);

  /* calculate dynamic collision around detection area */
  const bool has_collision = checkCollision(
    lanelet_map_ptr, *path, detection_lanelets, adjacent_lanelets, intersection_area, objects_ptr,
    closest_idx, stuck_vehicle_detect_area);

  /* calculate final stop lines */
  int stop_line_idx_final =
    stop_lines_idx_opt.has_value() ? stop_lines_idx_opt.value().stop_line : -1;
  int pass_judge_line_idx_final =
    stop_lines_idx_opt.has_value() ? stop_lines_idx_opt.value().pass_judge_line : -1;
  if (external_go) {
    is_entry_prohibited = false;
  } else if (external_stop) {
    is_entry_prohibited = true;
  } else if (is_stuck || has_collision) {
    is_entry_prohibited = true;
    const double dist_stuck_stopline = motion_utils::calcSignedArcLength(
      path->points, path->points.at(stuck_line_idx).point.pose.position,
      path->points.at(closest_idx).point.pose.position);
    const double eps = 1e-1;  // NOTE: check if sufficiently over the stuck stopline
    const bool is_over_stuck_stopline =
      util::isOverTargetIndex(*path, closest_idx, current_pose.pose, stuck_line_idx) &&
      dist_stuck_stopline > eps;
    if (is_stuck && !is_over_stuck_stopline) {
      stop_line_idx_final = stuck_line_idx;
      pass_judge_line_idx_final = stuck_line_idx;
    } else if (
      ((is_stuck && is_over_stuck_stopline) || has_collision) && stop_lines_idx_opt.has_value()) {
      stop_line_idx_final = stop_lines_idx_opt.value().stop_line;
      pass_judge_line_idx_final = stop_lines_idx_opt.value().pass_judge_line;
    }
  }

  if (stop_line_idx_final == -1) {
    RCLCPP_DEBUG(logger_, "detection_area is empty, no plan needed");
    RCLCPP_DEBUG(logger_, "===== plan end =====");
    setSafe(true);
    setDistance(std::numeric_limits<double>::lowest());
    return false;
  }

  state_machine_.setStateWithMarginTime(
    is_entry_prohibited ? IntersectionModule::State::STOP : IntersectionModule::State::GO,
    logger_.get_child("state_machine"), *clock_);

  setSafe(state_machine_.getState() == IntersectionModule::State::GO);
  setDistance(motion_utils::calcSignedArcLength(
    path->points, planner_data_->current_pose.pose.position,
    path->points.at(stop_line_idx_final).point.pose.position));

  if (!isActivated()) {
    // if RTC says intersection entry is 'dangerous', insert stop_line(v == 0.0) in this block
    is_go_out_ = false;

    constexpr double v = 0.0;
    planning_utils::setVelocityFrom(stop_line_idx_final, v, path);
    debug_data_.stop_required = true;
    const double base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;
    debug_data_.stop_wall_pose = util::getAheadPose(stop_line_idx_final, base_link2front, *path);
    debug_data_.stop_point_pose = path->points.at(stop_line_idx_final).point.pose;
    debug_data_.judge_point_pose = path->points.at(pass_judge_line_idx_final).point.pose;

    /* get stop point and stop factor */
    tier4_planning_msgs::msg::StopFactor stop_factor;
    stop_factor.stop_pose = debug_data_.stop_point_pose;
    const auto stop_factor_conflict = planning_utils::toRosPoints(debug_data_.conflicting_targets);
    const auto stop_factor_stuck = planning_utils::toRosPoints(debug_data_.stuck_targets);
    stop_factor.stop_factor_points =
      planning_utils::concatVector(stop_factor_conflict, stop_factor_stuck);
    planning_utils::appendStopReason(stop_factor, stop_reason);

    RCLCPP_DEBUG(logger_, "not activated. stop at the line.");
    RCLCPP_DEBUG(logger_, "===== plan end =====");
    return true;
  }

  is_go_out_ = true;
  RCLCPP_DEBUG(logger_, "===== plan end =====");
  return true;
}

void IntersectionModule::cutPredictPathWithDuration(
  autoware_auto_perception_msgs::msg::PredictedObjects * objects_ptr, const double time_thr) const
{
  const rclcpp::Time current_time = clock_->now();
  for (auto & object : objects_ptr->objects) {                         // each objects
    for (auto & predicted_path : object.kinematics.predicted_paths) {  // each predicted paths
      const auto origin_path = predicted_path;
      predicted_path.path.clear();

      for (size_t k = 0; k < origin_path.path.size(); ++k) {  // each path points
        const auto & predicted_pose = origin_path.path.at(k);
        const auto predicted_time =
          rclcpp::Time(objects_ptr->header.stamp) +
          rclcpp::Duration(origin_path.time_step) * static_cast<double>(k);
        if ((predicted_time - current_time).seconds() < time_thr) {
          predicted_path.path.push_back(predicted_pose);
        }
      }
    }
  }
}

bool IntersectionModule::checkCollision(
  lanelet::LaneletMapConstPtr lanelet_map_ptr,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const lanelet::ConstLanelets & detection_area_lanelets,
  const lanelet::ConstLanelets & adjacent_lanelets,
  const std::optional<Polygon2d> & intersection_area,
  const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr objects_ptr,
  const int closest_idx, const Polygon2d & stuck_vehicle_detect_area)
{
  using lanelet::utils::getArcCoordinates;
  using lanelet::utils::getPolygonFromArcLength;

  /* generate ego-lane polygon */
  const auto ego_lane_poly = lanelet_map_ptr->laneletLayer.get(module_id_).polygon2d();
  Polygon2d ego_poly{};
  for (const auto & p : ego_lane_poly) {
    ego_poly.outer().emplace_back(p.x(), p.y());
  }
  lanelet::ConstLanelets ego_lane_with_next_lane = getEgoLaneWithNextLane(lanelet_map_ptr, path);
  lanelet::ConstLanelet closest_lanelet;
  lanelet::utils::query::getClosestLanelet(
    ego_lane_with_next_lane, tier4_autoware_utils::getPose(path.points.at(closest_idx).point),
    &closest_lanelet);

  /* extract target objects */
  autoware_auto_perception_msgs::msg::PredictedObjects target_objects;
  target_objects.header = objects_ptr->header;
  for (const auto & object : objects_ptr->objects) {
    // ignore non-vehicle type objects, such as pedestrian.
    if (!isTargetCollisionVehicleType(object)) {
      continue;
    }

    // ignore vehicle in ego-lane && behind ego
    const auto object_pose = object.kinematics.initial_pose_with_covariance.pose;
    const bool is_in_ego_lane = bg::within(to_bg2d(object_pose.position), ego_poly);
    if (is_in_ego_lane) {
      if (!planning_utils::isAheadOf(object_pose, planner_data_->current_pose.pose)) {
        continue;
      }
      if (
        planner_param_.enable_front_car_decel_prediction &&
        checkFrontVehicleDeceleration(
          ego_lane_with_next_lane, closest_lanelet, stuck_vehicle_detect_area, object))
        return true;
    }

    // check direction of objects
    const auto object_direction = getObjectPoseWithVelocityDirection(object.kinematics);
    if (intersection_area) {
      const auto obj_poly = toFootprintPolygon(object);
      const auto intersection_area_2d = intersection_area.value();
      const auto is_in_intersection_area = bg::within(obj_poly, intersection_area_2d);
      const auto is_in_adjacent_lanelets = checkAngleForTargetLanelets(
        object_direction, adjacent_lanelets, planner_param_.detection_area_margin);
      if (is_in_adjacent_lanelets) continue;
      if (is_in_intersection_area) {
        target_objects.objects.push_back(object);
      } else if (checkAngleForTargetLanelets(
                   object_direction, detection_area_lanelets,
                   planner_param_.detection_area_margin)) {
        target_objects.objects.push_back(object);
      }
    } else if (checkAngleForTargetLanelets(
                 object_direction, detection_area_lanelets, planner_param_.detection_area_margin)) {
      // intersection_area is not available, use detection_area_with_margin as before
      target_objects.objects.push_back(object);
    }
  }

  /* check collision between target_objects predicted path and ego lane */

  // cut the predicted path at passing_time
  const auto time_distance_array = calcIntersectionPassingTime(path, closest_idx, lane_id_);
  const double passing_time = time_distance_array.back().first;
  cutPredictPathWithDuration(&target_objects, passing_time);

  const auto closest_arc_coords = getArcCoordinates(
    ego_lane_with_next_lane, tier4_autoware_utils::getPose(path.points.at(closest_idx).point));
  const double distance_until_intersection =
    calcDistanceUntilIntersectionLanelet(lanelet_map_ptr, path, closest_idx);
  const double base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;

  // check collision between predicted_path and ego_area
  bool collision_detected = false;
  for (const auto & object : target_objects.objects) {
    bool has_collision = false;
    for (const auto & predicted_path : object.kinematics.predicted_paths) {
      if (predicted_path.confidence < planner_param_.min_predicted_path_confidence) {
        // ignore the predicted path with too low confidence
        continue;
      }

      std::vector<geometry_msgs::msg::Pose> predicted_poses;
      for (const auto & pose : predicted_path.path) {
        predicted_poses.push_back(pose);
      }
      has_collision = bg::intersects(ego_poly, to_bg2d(predicted_poses));
      if (has_collision) {
        const auto first_itr = std::adjacent_find(
          predicted_path.path.cbegin(), predicted_path.path.cend(),
          [&ego_poly](const auto & a, const auto & b) {
            return bg::intersects(ego_poly, LineString2d{to_bg2d(a), to_bg2d(b)});
          });
        const auto last_itr = std::adjacent_find(
          predicted_path.path.crbegin(), predicted_path.path.crend(),
          [&ego_poly](const auto & a, const auto & b) {
            return bg::intersects(ego_poly, LineString2d{to_bg2d(a), to_bg2d(b)});
          });
        const double ref_object_enter_time =
          static_cast<double>(first_itr - predicted_path.path.begin()) *
          rclcpp::Duration(predicted_path.time_step).seconds();
        auto start_time_distance_itr = time_distance_array.begin();
        if (ref_object_enter_time - planner_param_.collision_start_margin_time > 0) {
          start_time_distance_itr = std::lower_bound(
            time_distance_array.begin(), time_distance_array.end(),
            ref_object_enter_time - planner_param_.collision_start_margin_time,
            [](const auto & a, const double b) { return a.first < b; });
          if (start_time_distance_itr == time_distance_array.end()) {
            continue;
          }
        }
        const double ref_object_exit_time =
          static_cast<double>(last_itr.base() - predicted_path.path.begin()) *
          rclcpp::Duration(predicted_path.time_step).seconds();
        auto end_time_distance_itr = std::lower_bound(
          time_distance_array.begin(), time_distance_array.end(),
          ref_object_exit_time + planner_param_.collision_end_margin_time,
          [](const auto & a, const double b) { return a.first < b; });
        if (end_time_distance_itr == time_distance_array.end()) {
          end_time_distance_itr = time_distance_array.end() - 1;
        }
        const double start_arc_length = std::max(
          0.0, closest_arc_coords.length + (*start_time_distance_itr).second -
                 distance_until_intersection);
        const double end_arc_length = std::max(
          0.0, closest_arc_coords.length + (*end_time_distance_itr).second + base_link2front -
                 distance_until_intersection);
        const auto trimmed_ego_polygon =
          getPolygonFromArcLength(ego_lane_with_next_lane, start_arc_length, end_arc_length);

        Polygon2d polygon{};
        for (const auto & p : trimmed_ego_polygon) {
          polygon.outer().emplace_back(p.x(), p.y());
        }

        polygon.outer().emplace_back(polygon.outer().front());

        debug_data_.candidate_collision_ego_lane_polygon = toGeomMsg(polygon);

        for (auto itr = first_itr; itr != last_itr.base(); ++itr) {
          const auto footprint_polygon = toPredictedFootprintPolygon(object, *itr);
          debug_data_.candidate_collision_object_polygons.emplace_back(
            toGeomMsg(footprint_polygon));
          if (bg::intersects(polygon, footprint_polygon)) {
            collision_detected = true;
            break;
          }
        }
        if (collision_detected) {
          debug_data_.conflicting_targets.objects.push_back(object);
          break;
        }
      }
    }
  }

  return collision_detected;
}

Polygon2d IntersectionModule::generateEgoIntersectionLanePolygon(
  lanelet::LaneletMapConstPtr lanelet_map_ptr,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const int closest_idx,
  const double extra_dist, const double ignore_dist) const
{
  using lanelet::utils::getArcCoordinates;
  using lanelet::utils::getLaneletLength3d;
  using lanelet::utils::getPolygonFromArcLength;
  using lanelet::utils::to2D;

  lanelet::ConstLanelets ego_lane_with_next_lane = getEgoLaneWithNextLane(lanelet_map_ptr, path);

  const double intersection_exit_length = getLaneletLength3d(ego_lane_with_next_lane.front());

  const auto closest_arc_coords = getArcCoordinates(
    ego_lane_with_next_lane, tier4_autoware_utils::getPose(path.points.at(closest_idx).point));

  const double start_arc_length = intersection_exit_length - ignore_dist > closest_arc_coords.length
                                    ? intersection_exit_length - ignore_dist
                                    : closest_arc_coords.length;

  const double end_arc_length = getLaneletLength3d(ego_lane_with_next_lane.front()) + extra_dist;

  const auto target_polygon =
    to2D(getPolygonFromArcLength(ego_lane_with_next_lane, start_arc_length, end_arc_length))
      .basicPolygon();

  Polygon2d polygon{};

  if (target_polygon.empty()) {
    return polygon;
  }

  for (const auto & p : target_polygon) {
    polygon.outer().emplace_back(p.x(), p.y());
  }

  polygon.outer().emplace_back(polygon.outer().front());

  return polygon;
}

TimeDistanceArray IntersectionModule::calcIntersectionPassingTime(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const int closest_idx,
  const int objective_lane_id) const
{
  static constexpr double k_minimum_velocity = 1e-01;

  double dist_sum = 0.0;
  double passing_time = 0.0;
  int assigned_lane_found = false;

  // crop intersection part of the path, and set the reference velocity to intersection_velocity for
  // ego's ttc
  PathWithLaneId reference_path;
  for (size_t i = closest_idx; i < path.points.size(); ++i) {
    auto reference_point = path.points.at(i);
    reference_point.point.longitudinal_velocity_mps = planner_param_.intersection_velocity;
    reference_path.points.push_back(reference_point);
    bool has_objective_lane_id = util::hasLaneId(path.points.at(i), objective_lane_id);
    if (assigned_lane_found && !has_objective_lane_id) {
      break;
    }
    assigned_lane_found = has_objective_lane_id;
  }
  if (!assigned_lane_found) {
    return {{0.0, 0.0}};  // has already passed the intersection.
  }

  // apply smoother to reference velocity
  PathWithLaneId smoothed_reference_path = reference_path;
  if (!smoothPath(reference_path, smoothed_reference_path, planner_data_)) {
    RCLCPP_WARN(logger_, "smoothPath failed");
  }

  // calculate when ego is going to reach each (interpolated) points on the path
  TimeDistanceArray time_distance_array{};
  dist_sum = 0.0;
  passing_time = 0.0;
  time_distance_array.emplace_back(passing_time, dist_sum);
  for (size_t i = 1; i < smoothed_reference_path.points.size(); ++i) {
    const auto & p1 = smoothed_reference_path.points.at(i - 1);
    const auto & p2 = smoothed_reference_path.points.at(i);

    const double dist = tier4_autoware_utils::calcDistance2d(p1, p2);
    dist_sum += dist;

    // use average velocity between p1 and p2
    const double average_velocity =
      (p1.point.longitudinal_velocity_mps + p2.point.longitudinal_velocity_mps) / 2.0;
    passing_time +=
      (dist / std::max<double>(k_minimum_velocity, average_velocity));  // to avoid zero-division

    time_distance_array.emplace_back(passing_time, dist_sum);
  }
  RCLCPP_DEBUG(logger_, "intersection dist = %f, passing_time = %f", dist_sum, passing_time);

  return time_distance_array;
}

bool IntersectionModule::checkStuckVehicleInIntersection(
  const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr objects_ptr,
  const Polygon2d & stuck_vehicle_detect_area) const
{
  debug_data_.stuck_vehicle_detect_area = toGeomMsg(stuck_vehicle_detect_area);

  for (const auto & object : objects_ptr->objects) {
    if (!isTargetStuckVehicleType(object)) {
      continue;  // not target vehicle type
    }
    const auto obj_v = std::fabs(object.kinematics.initial_twist_with_covariance.twist.linear.x);
    if (obj_v > planner_param_.stuck_vehicle_vel_thr) {
      continue;  // not stop vehicle
    }

    // check if the footprint is in the stuck detect area
    const Polygon2d obj_footprint = toFootprintPolygon(object);
    const bool is_in_stuck_area = !bg::disjoint(obj_footprint, stuck_vehicle_detect_area);
    if (is_in_stuck_area) {
      RCLCPP_DEBUG(logger_, "stuck vehicle found.");
      debug_data_.stuck_targets.objects.push_back(object);
      return true;
    }
  }
  return false;
}

Polygon2d IntersectionModule::toFootprintPolygon(
  const autoware_auto_perception_msgs::msg::PredictedObject & object) const
{
  Polygon2d obj_footprint;
  if (object.shape.type == autoware_auto_perception_msgs::msg::Shape::POLYGON) {
    obj_footprint = toBoostPoly(object.shape.footprint);
  } else {
    // cylinder type is treated as square-polygon
    obj_footprint =
      obj2polygon(object.kinematics.initial_pose_with_covariance.pose, object.shape.dimensions);
  }
  return obj_footprint;
}

Polygon2d IntersectionModule::toPredictedFootprintPolygon(
  const autoware_auto_perception_msgs::msg::PredictedObject & object,
  const geometry_msgs::msg::Pose & predicted_pose) const
{
  return obj2polygon(predicted_pose, object.shape.dimensions);
}

bool IntersectionModule::isTargetCollisionVehicleType(
  const autoware_auto_perception_msgs::msg::PredictedObject & object) const
{
  if (
    object.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::CAR ||
    object.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::BUS ||
    object.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::TRUCK ||
    object.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::TRAILER ||
    object.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::MOTORCYCLE ||
    object.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::BICYCLE) {
    return true;
  }
  return false;
}

bool IntersectionModule::isTargetStuckVehicleType(
  const autoware_auto_perception_msgs::msg::PredictedObject & object) const
{
  if (
    object.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::CAR ||
    object.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::BUS ||
    object.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::TRUCK ||
    object.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::TRAILER ||
    object.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::MOTORCYCLE) {
    return true;
  }
  return false;
}
void IntersectionModule::StateMachine::setStateWithMarginTime(
  State state, rclcpp::Logger logger, rclcpp::Clock & clock)
{
  /* same state request */
  if (state_ == state) {
    start_time_ = nullptr;  // reset timer
    return;
  }

  /* GO -> STOP */
  if (state == State::STOP) {
    state_ = State::STOP;
    start_time_ = nullptr;  // reset timer
    return;
  }

  /* STOP -> GO */
  if (state == State::GO) {
    if (start_time_ == nullptr) {
      start_time_ = std::make_shared<rclcpp::Time>(clock.now());
    } else {
      const double duration = (clock.now() - *start_time_).seconds();
      if (duration > margin_time_) {
        state_ = State::GO;
        start_time_ = nullptr;  // reset timer
      }
    }
    return;
  }

  RCLCPP_ERROR(logger, "Unsuitable state. ignore request.");
}

void IntersectionModule::StateMachine::setState(State state) { state_ = state; }

void IntersectionModule::StateMachine::setMarginTime(const double t) { margin_time_ = t; }

IntersectionModule::State IntersectionModule::StateMachine::getState() { return state_; }

bool IntersectionModule::isTargetExternalInputStatus(const int target_status)
{
  return planner_data_->external_intersection_status_input &&
         planner_data_->external_intersection_status_input.get().status == target_status &&
         (clock_->now() - planner_data_->external_intersection_status_input.get().header.stamp)
             .seconds() < planner_param_.external_input_timeout;
}

bool IntersectionModule::checkAngleForTargetLanelets(
  const geometry_msgs::msg::Pose & pose, const lanelet::ConstLanelets & target_lanelets,
  const double margin)
{
  for (const auto & ll : target_lanelets) {
    if (!lanelet::utils::isInLanelet(pose, ll, margin)) {
      continue;
    }
    const double ll_angle = lanelet::utils::getLaneletAngle(ll, pose.position);
    const double pose_angle = tf2::getYaw(pose.orientation);
    const double angle_diff = tier4_autoware_utils::normalizeRadian(ll_angle - pose_angle);
    if (std::fabs(angle_diff) < planner_param_.detection_area_angle_thr) {
      return true;
    }
  }
  return false;
}

lanelet::ConstLanelets IntersectionModule::getEgoLaneWithNextLane(
  lanelet::LaneletMapConstPtr lanelet_map_ptr,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path) const
{
  const auto & assigned_lanelet = lanelet_map_ptr->laneletLayer.get(lane_id_);
  const auto last_itr = std::find_if(
    path.points.crbegin(), path.points.crend(),
    [this](const auto & p) { return p.lane_ids.front() == lane_id_; });
  lanelet::ConstLanelets ego_lane_with_next_lane;
  if (last_itr.base() != path.points.end()) {
    const auto & next_lanelet =
      lanelet_map_ptr->laneletLayer.get((*last_itr.base()).lane_ids.front());
    ego_lane_with_next_lane = {assigned_lanelet, next_lanelet};
  } else {
    ego_lane_with_next_lane = {assigned_lanelet};
  }
  return ego_lane_with_next_lane;
}

double IntersectionModule::calcDistanceUntilIntersectionLanelet(
  lanelet::LaneletMapConstPtr lanelet_map_ptr,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const size_t closest_idx) const
{
  const auto & assigned_lanelet = lanelet_map_ptr->laneletLayer.get(lane_id_);
  const auto intersection_first_itr = std::find_if(
    path.points.cbegin(), path.points.cend(),
    [this](const auto & p) { return p.lane_ids.front() == lane_id_; });
  if (
    intersection_first_itr == path.points.begin() || intersection_first_itr == path.points.end()) {
    return 0.0;
  }
  const auto dst_idx = std::distance(path.points.begin(), intersection_first_itr) - 1;

  if (closest_idx > static_cast<size_t>(dst_idx)) {
    return 0.0;
  }

  double distance = motion_utils::calcSignedArcLength(path.points, closest_idx, dst_idx);
  const auto & lane_first_point = assigned_lanelet.centerline2d().front();
  distance += std::hypot(
    path.points.at(dst_idx).point.pose.position.x - lane_first_point.x(),
    path.points.at(dst_idx).point.pose.position.y - lane_first_point.y());
  return distance;
}

bool IntersectionModule::checkFrontVehicleDeceleration(
  lanelet::ConstLanelets & ego_lane_with_next_lane, lanelet::ConstLanelet & closest_lanelet,
  const Polygon2d & stuck_vehicle_detect_area,
  const autoware_auto_perception_msgs::msg::PredictedObject & object) const
{
  const auto & object_pose = object.kinematics.initial_pose_with_covariance.pose;
  // consider vehicle in ego-lane && in front of ego
  const auto lon_vel = object.kinematics.initial_twist_with_covariance.twist.linear.x;
  const double object_decel = planner_param_.assumed_front_car_decel;  // NOTE: this is positive
  const double stopping_distance = lon_vel * lon_vel / (2 * object_decel);

  std::vector<geometry_msgs::msg::Point> center_points;
  for (auto && p : ego_lane_with_next_lane[0].centerline())
    center_points.push_back(std::move(lanelet::utils::conversion::toGeomMsgPt(p)));
  for (auto && p : ego_lane_with_next_lane[1].centerline())
    center_points.push_back(std::move(lanelet::utils::conversion::toGeomMsgPt(p)));
  const double lat_offset =
    std::fabs(motion_utils::calcLateralOffset(center_points, object_pose.position));
  // get the nearest centerpoint to object
  std::vector<double> dist_obj_center_points;
  for (const auto & p : center_points)
    dist_obj_center_points.push_back(tier4_autoware_utils::calcDistance2d(object_pose.position, p));
  const int obj_closest_centerpoint_idx = std::distance(
    dist_obj_center_points.begin(),
    std::min_element(dist_obj_center_points.begin(), dist_obj_center_points.end()));
  // find two center_points whose distances from `closest_centerpoint` cross stopping_distance
  double acc_dist_prev = 0.0, acc_dist = 0.0;
  auto p1 = center_points[obj_closest_centerpoint_idx];
  auto p2 = center_points[obj_closest_centerpoint_idx];
  for (unsigned i = obj_closest_centerpoint_idx; i < center_points.size() - 1; ++i) {
    p1 = center_points[i];
    p2 = center_points[i + 1];
    acc_dist_prev = acc_dist;
    const auto arc_position_p1 =
      lanelet::utils::getArcCoordinates(ego_lane_with_next_lane, util::toPose(p1));
    const auto arc_position_p2 =
      lanelet::utils::getArcCoordinates(ego_lane_with_next_lane, util::toPose(p2));
    const double delta = arc_position_p2.length - arc_position_p1.length;
    acc_dist += delta;
    if (acc_dist > stopping_distance) {
      break;
    }
  }
  // if stopping_distance >= center_points, stopping_point is center_points[end]
  const double ratio = (acc_dist <= stopping_distance)
                         ? 0.0
                         : (acc_dist - stopping_distance) / (stopping_distance - acc_dist_prev);
  // linear interpolation
  geometry_msgs::msg::Point stopping_point;
  stopping_point.x = (p1.x * ratio + p2.x) / (1 + ratio);
  stopping_point.y = (p1.y * ratio + p2.y) / (1 + ratio);
  stopping_point.z = (p1.z * ratio + p2.z) / (1 + ratio);
  const double lane_yaw = lanelet::utils::getLaneletAngle(closest_lanelet, stopping_point);
  stopping_point.x += lat_offset * std::cos(lane_yaw + M_PI / 2.0);
  stopping_point.y += lat_offset * std::sin(lane_yaw + M_PI / 2.0);

  // calculate footprint of predicted stopping pose
  autoware_auto_perception_msgs::msg::PredictedObject predicted_object = object;
  predicted_object.kinematics.initial_pose_with_covariance.pose.position = stopping_point;
  predicted_object.kinematics.initial_pose_with_covariance.pose.orientation =
    tier4_autoware_utils::createQuaternionFromRPY(0, 0, lane_yaw);
  Polygon2d predicted_obj_footprint = toFootprintPolygon(predicted_object);
  const bool is_in_stuck_area = !bg::disjoint(predicted_obj_footprint, stuck_vehicle_detect_area);
  debug_data_.predicted_obj_pose.position = stopping_point;
  debug_data_.predicted_obj_pose.orientation =
    tier4_autoware_utils::createQuaternionFromRPY(0, 0, lane_yaw);

  if (is_in_stuck_area) {
    return true;
  }
  return false;
}

void IntersectionModule::showOccupancyGridImage(
  const int lane_id, const nav_msgs::msg::OccupancyGrid & occ_grid,
  const std::vector<lanelet::CompoundPolygon3d> & detection_areas,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  [[maybe_unused]] const std::vector<util::DetectionLaneDivision> & lane_divisions) const
{
  const int width = occ_grid.info.width;
  const int height = occ_grid.info.height;
  const double reso = occ_grid.info.resolution;
  const auto & origin = occ_grid.info.origin.position;

  // NOTE: interesting area is set to 0 for later masking
  cv::Mat detection_mask(width, height, CV_8UC1, cv::Scalar(0));
  cv::Mat unknown_mask(width, height, CV_8UC1, cv::Scalar(0));

  // (1) prepare detection area mask
  Polygon2d grid_poly;
  grid_poly.outer().emplace_back(origin.x, origin.y);
  grid_poly.outer().emplace_back(origin.x + (width - 1) * reso, origin.y);
  grid_poly.outer().emplace_back(origin.x + (width - 1) * reso, origin.y + (height - 1) * reso);
  grid_poly.outer().emplace_back(origin.x, origin.y + (height - 1) * reso);
  grid_poly.outer().emplace_back(origin.x, origin.y);
  bg::correct(grid_poly);

  std::vector<std::vector<cv::Point>> detection_area_cv_polygons;
  for (const auto & detection_area : detection_areas) {
    const auto area2d = lanelet::utils::to2D(detection_area);
    Polygon2d area2d_poly;
    for (const auto & p : area2d) {
      area2d_poly.outer().emplace_back(p.x(), p.y());
    }
    area2d_poly.outer().push_back(area2d_poly.outer().front());
    bg::correct(area2d_poly);
    std::vector<Polygon2d> common_areas;
    bg::intersection(area2d_poly, grid_poly, common_areas);
    if (common_areas.empty()) {
      continue;
    }
    for (size_t i = 0; i < common_areas.size(); ++i) {
      common_areas[i].outer().push_back(common_areas[i].outer().front());
      bg::correct(common_areas[i]);
    }
    for (const auto & common_area : common_areas) {
      std::vector<cv::Point> detection_area_cv_polygon;
      for (const auto & p : common_area.outer()) {
        const int idx_x = static_cast<int>((p.x() - origin.x) / reso);
        const int idx_y = static_cast<int>((p.y() - origin.y) / reso);
        detection_area_cv_polygon.emplace_back(idx_x, height - 1 - idx_y);
      }
      detection_area_cv_polygons.push_back(detection_area_cv_polygon);
    }
  }
  for (const auto & poly : detection_area_cv_polygons) {
    cv::fillPoly(detection_mask, poly, cv::Scalar(255), cv::LINE_AA);
  }

  // (2) prepare unknown mask
  // In OpenCV the pixel at (X=x, Y=y) (with left-upper origin) is accesed by img[y, x]
  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      const int idx = y * width + x;
      const unsigned char intensity = occ_grid.data.at(idx);
      if (43 <= intensity && intensity < 58) {
        unknown_mask.at<unsigned char>(height - 1 - y, x) = 255;
      }
    }
  }

  // (3) occlusion mask
  cv::Mat occlusion_mask(width, height, CV_8UC1, cv::Scalar(0));
  cv::bitwise_and(detection_mask, unknown_mask, occlusion_mask);

  // (4) create distance grid
  // value: 0 - 254: signed distance representing [distamce_min, distance_max]
  // 255: undefined value
  const double distance_max = std::hypot(width * reso / 2.0, height * reso / 2.0);
  const double distance_min = -distance_max;
  auto dist2pixel = [=](const double dist) {
    return std::min(
      254, static_cast<int>((dist - distance_min) / (distance_max - distance_min) * 254));
  };
  auto pixel2dist = [=](const int pixel) {
    return pixel * 1.0 / 254 * (distance_max - distance_min) + distance_min;
  };

  auto coord2index = [&](const double x, const double y) {
    const int idx_x = (x - origin.x) / reso;
    const int idx_y = (y - origin.y) / reso;
    if (idx_x < 0 || idx_x >= width) return std::make_tuple(false, -1, -1);
    if (idx_y < 0 || idx_y >= height) return std::make_tuple(false, -1, -1);
    return std::make_tuple(true, idx_x, idx_y);
  };

  cv::Mat distance_grid(width, height, CV_8UC1, cv::Scalar(255));

  // (4.1) first mark trajectory cell as 0.0
  const auto intersection_interval_opt = util::findLaneIdInterval(path, lane_id);
  if (!intersection_interval_opt.has_value()) {
    return;
  }
  const int zero_dist_pixel = dist2pixel(0.0);
  const auto [lane_start, lane_end] = intersection_interval_opt.value();
  for (size_t i = lane_start; i <= lane_end; ++i) {
    const auto & path_pos = path.points.at(i).point.pose.position;
    const int idx_x = (path_pos.x - origin.x) / reso;
    const int idx_y = (path_pos.y - origin.y) / reso;
    if (idx_x < 0 || idx_x >= width) continue;
    if (idx_y < 0 || idx_y >= height) continue;
    distance_grid.at<unsigned char>(height - 1 - idx_y, idx_x) = zero_dist_pixel;
  }

  [[maybe_unused]] auto bresenham_line = [&](
                                           const int x0, const int y0, const int x1, const int y1,
                                           const std::optional<double> steep_opt, const int pixel) {
    if (!steep_opt.has_value()) {
      // x0 == x1
      for (int y = y0; y <= y1; y++) {
        const int cur = distance_grid.at<unsigned char>(height - 1 - y, x0);
        if (cur == 255) distance_grid.at<unsigned char>(height - 1 - y, x0) = pixel;
      }
      return;
    }
    if (y0 == y1) {
      for (int x = x0; x <= x1; x++) {
        const int cur = distance_grid.at<unsigned char>(height - 1 - y0, x);
        if (cur == 255) distance_grid.at<unsigned char>(height - 1 - y0, x) = pixel;
      }
      return;
    }

    const double steep = steep_opt.value();
    const int x_dir = (x0 < x1) ? 1 : -1;
    const int y_dir = (steep * x_dir) > 0 ? 1 : -1;
    int x = x0, y = y0;
    double error = 0;
    while (x != x1) {
      const int cur = distance_grid.at<unsigned char>(height - 1 - y, x);
      if (cur == 255) distance_grid.at<unsigned char>(height - 1 - y, x) = pixel;
      error += steep;
      if (error >= 0.5) {
        y = y + y_dir;
        error = error - 1.0;
      }
      x = x + x_dir;
    }
  };

  for (const auto & lane_division : lane_divisions) {
    const auto & divisions = lane_division.divisions;
    for (const auto & division : divisions) {
      bool is_in_grid = false;
      double acc_dist = 0.0;
      bool cost_defined = false;
      std::optional<double> prev_checkpoint_x = std::nullopt, prev_checkpoint_y = std::nullopt;
      std::vector<lanelet::ConstPoint2d> prev_checkpoints;
      for (const auto & point : division) {
        const auto [valid, idx_x, idx_y] = coord2index(point.x(), point.y());
        if (is_in_grid && !valid) break;  // exited grid just now
        if (!is_in_grid && !valid) continue;
        const int pixel = distance_grid.at<unsigned char>(height - 1 - idx_y, idx_x);
        if (!is_in_grid && valid) {
          // entered grid
          is_in_grid = true;
          if (pixel == 255) {
            // cannot decide signed distance from PathPointWithLaneId at this momemt
            cost_defined = false;
            prev_checkpoints.push_back(point);
          } else {
            cost_defined = true;
            acc_dist = pixel2dist(pixel);
            prev_checkpoint_x = point.x();
            prev_checkpoint_y = point.y();
          }
          continue;
        }
        if (is_in_grid && valid) {
          if (pixel == 255 && !cost_defined) {
            // still cannot decide signed distance from PathPointWithLaneId
            prev_checkpoints.push_back(point);
            continue;
          }
          if (pixel == 255 && cost_defined) {
            const double dy = point.y() - prev_checkpoint_y.value(),
                         dx = point.x() - prev_checkpoint_x.value();
            [[maybe_unused]] const std::optional<double> steep =
              (fabs(dx) < 0.05) ? std::nullopt : std::make_optional(dy / dx);
            [[maybe_unused]] const auto [valid, prev_idx_x, prev_idx_y] =
              coord2index(prev_checkpoint_x.value(), prev_checkpoint_y.value());
            // bresenham_line(prev_idx_x, prev_idx_y, idx_x, idx_y, steep, dist2pixel(acc_dist));
            acc_dist += std::hypot(dy, dx);
            distance_grid.at<unsigned char>(height - 1 - idx_y, idx_x) = dist2pixel(acc_dist);
            prev_checkpoint_x = point.x();
            prev_checkpoint_y = point.y();
            continue;
          }
          if (pixel == zero_dist_pixel) {
            // actually, cells with minus signed distance are not negligible
            double acc_dist_prev = 0.0;
            auto prev_point = point;
            for (const auto & cur_point : prev_checkpoints) {
              const double dx = cur_point.x() - prev_point.x(), dy = cur_point.y() - prev_point.y();
              [[maybe_unused]] const std::optional<double> steep =
                (fabs(dx) < 0.05) ? std::nullopt : std::make_optional(dy / dx);
              const double dd = std::hypot(dy, dx);
              [[maybe_unused]] const auto [valid_prev, prev_idx_x, prev_idx_y] =
                coord2index(prev_point.x(), prev_point.y());
              const auto [valid, cur_idx_x, cur_idx_y] = coord2index(cur_point.x(), cur_point.y());
              // bresenham_line(
              //   prev_idx_x, prev_idx_y, cur_idx_x, cur_idx_y, steep, dist2pixel(acc_dist_prev));
              acc_dist_prev -= dd;
              distance_grid.at<unsigned char>(height - 1 - cur_idx_y, cur_idx_x) =
                dist2pixel(acc_dist_prev);
              prev_point = cur_point;
            }
            acc_dist = 0.0;
            cost_defined = true;
            prev_checkpoint_x = point.x();
            prev_checkpoint_y = point.y();
            continue;
          } else {
            // pixel != 255 and pixel != zero_dist_pixel
            // Dynamic Programming to lower cost
            if (!prev_checkpoint_x.has_value()) {
              // assert(not cost_defined)
              acc_dist = pixel2dist(pixel);
              cost_defined = true;
              prev_checkpoint_x = point.x();
              prev_checkpoint_y = point.y();
            } else {
              // assert(cost_defined)
              const double cur_dist = pixel2dist(pixel);
              const double new_dist = acc_dist + std::hypot(
                                                   point.x() - prev_checkpoint_x.value(),
                                                   point.y() - prev_checkpoint_y.value());
              if (new_dist < cur_dist) {
                acc_dist = new_dist;
                distance_grid.at<unsigned char>(height - 1 - idx_y, idx_x) = dist2pixel(acc_dist);
              }
              cost_defined = true;
              prev_checkpoint_x = point.x();
              prev_checkpoint_y = point.y();
            }
          }
        }
      }
    }
  }
  for (int i = 0; i < width; ++i) {
    for (int j = 0; j < height; ++j) {
      if (distance_grid.at<unsigned char>(j, i) == 255) {
        distance_grid.at<unsigned char>(j, i) = 0;
      }
    }
  }
  cv::Mat distance_grid_cropped;
  cv::bitwise_and(distance_grid, occlusion_mask, distance_grid_cropped);
  cv::Mat distance_grid_heatmap;
  cv::applyColorMap(distance_grid_cropped, distance_grid_heatmap, cv::COLORMAP_JET);
  cv::namedWindow("distance_grid_viz", cv::WINDOW_NORMAL);
  cv::imshow("distance_grid_viz", distance_grid_heatmap);
  cv::waitKey(1);
}

}  // namespace behavior_velocity_planner
