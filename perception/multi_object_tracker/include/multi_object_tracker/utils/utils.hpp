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
//
//
// Author: v1.0 Yukihiro Saito
//

#ifndef MULTI_OBJECT_TRACKER__UTILS__UTILS_HPP_
#define MULTI_OBJECT_TRACKER__UTILS__UTILS_HPP_

#define EIGEN_MPL2_ONLY

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <autoware_auto_perception_msgs/msg/detected_object.hpp>
#include <autoware_auto_perception_msgs/msg/object_classification.hpp>
#include <autoware_auto_perception_msgs/msg/shape.hpp>
#include <geometry_msgs/msg/transform.hpp>

#include <tf2/utils.h>

#include <algorithm>

namespace utils
{
enum MSG_COV_IDX {
  X_X = 0,
  X_Y = 1,
  X_Z = 2,
  X_ROLL = 3,
  X_PITCH = 4,
  X_YAW = 5,
  Y_X = 6,
  Y_Y = 7,
  Y_Z = 8,
  Y_ROLL = 9,
  Y_PITCH = 10,
  Y_YAW = 11,
  Z_X = 12,
  Z_Y = 13,
  Z_Z = 14,
  Z_ROLL = 15,
  Z_PITCH = 16,
  Z_YAW = 17,
  ROLL_X = 18,
  ROLL_Y = 19,
  ROLL_Z = 20,
  ROLL_ROLL = 21,
  ROLL_PITCH = 22,
  ROLL_YAW = 23,
  PITCH_X = 24,
  PITCH_Y = 25,
  PITCH_Z = 26,
  PITCH_ROLL = 27,
  PITCH_PITCH = 28,
  PITCH_YAW = 29,
  YAW_X = 30,
  YAW_Y = 31,
  YAW_Z = 32,
  YAW_ROLL = 33,
  YAW_PITCH = 34,
  YAW_YAW = 35
};

enum BBOX_IDX {
  FRONT_SURFACE = 0,
  RIGHT_SURFACE = 1,
  REAR_SURFACE = 2,
  LEFT_SURFACE = 3,
  FRONT_R_CORNER = 4,
  REAR_R_CORNER = 5,
  REAR_L_CORNER = 6,
  FRONT_L_CORNER = 7,
  INSIDE = 8,
  INVALID = -1
};

/**
 * @brief check if object label belongs to "large vehicle"
 * @param label: input object label
 * @return True if object label means large vehicle
 */
bool isLargeVehicleLabel(const uint8_t label);

/**
 * @brief Determine the Nearest Corner or Surface of detected object observed from ego vehicle
 *
 * @param x: object x coordinate in map frame
 * @param y: object y coordinate in map frame
 * @param yaw: object yaw orientation in map frame
 * @param width: object bounding box width
 * @param length: object bounding box length
 * @param self_transform: Ego vehicle position in map frame
 * @return int index
 */
int getNearestCornerOrSurface(
  const double x, const double y, const double yaw, const double width, const double length,
  const geometry_msgs::msg::Transform & self_transform);

/**
 * @brief Get the Nearest Corner or Surface from detected object
 * @param object: input object
 * @param yaw: object yaw angle (after solved front and back uncertainty)
 * @param self_transform
 * @return nearest corner or surface index
 */
int getNearestCornerOrSurfaceFromObject(
  const autoware_auto_perception_msgs::msg::DetectedObject & object, const double & yaw,
  const geometry_msgs::msg::Transform & self_transform);

/**
 * @brief Calc bounding box center offset caused by shape change
 * @param dw: width update [m] =  w_new - w_old
 * @param dl: length update [m] = l_new - l_old
 * @param indx: nearest corner index
 * @return 2d offset vector caused by shape change
 */
Eigen::Vector2d calcOffsetVectorFromShapeChange(const double dw, const double dl, const int indx);

/**
 * @brief post-processing to recover bounding box center from tracking point and offset vector
 * @param x: x of tracking point estimated with ekf
 * @param y: y of tracking point estimated with ekf
 * @param yaw: yaw of tracking point estimated with ekf
 * @param dw: diff of width: w_estimated - w_input
 * @param dl: diff of length: l_estimated - l_input
 * @param indx: closest corner or surface index
 * @param tracking_offset: tracking offset between bounding box center and tracking point
 */
Eigen::Vector2d recoverFromTrackingPoint(
  const double x, const double y, const double yaw, const double dw, const double dl,
  const int indx, const Eigen::Vector2d & tracking_offset);

/**
 * @brief Convert input object center to tracking point based on nearest corner information
 * 1. update anchor offset vector, 2. offset input bbox based on tracking_offset vector and
 * prediction yaw angle
 * @param w: last input bounding box width
 * @param l: last input bounding box length
 * @param indx: last input bounding box closest corner index
 * @param input_object: input object bounding box
 * @param yaw: current yaw estimation
 * @param offset_object: output tracking measurement to feed ekf
 * @return nearest corner index(int)
 */
void calcAnchorPointOffset(
  const double w, const double l, const int indx,
  const autoware_auto_perception_msgs::msg::DetectedObject & input_object, const double & yaw,
  autoware_auto_perception_msgs::msg::DetectedObject & offset_object,
  Eigen::Vector2d & tracking_offset);

/**
 * @brief convert convex hull shape object to bounding box object
 * @param input_object: input convex hull objects
 * @param output_object: output bounding box objects
 */
void convertConvexHullToBoundingBox(
  const autoware_auto_perception_msgs::msg::DetectedObject & input_object,
  autoware_auto_perception_msgs::msg::DetectedObject & output_object);

}  // namespace utils

#endif  // MULTI_OBJECT_TRACKER__UTILS__UTILS_HPP_
