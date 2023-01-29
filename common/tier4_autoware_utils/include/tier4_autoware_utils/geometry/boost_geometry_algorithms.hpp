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

#ifndef TIER4_AUTOWARE_UTILS__GEOMETRY__BOOST_GEOMETRY_ALGORITHMS_HPP_
#define TIER4_AUTOWARE_UTILS__GEOMETRY__BOOST_GEOMETRY_ALGORITHMS_HPP_

#include <tier4_autoware_utils/geometry/boost_geometry.hpp>

#include <lanelet2_core/Forward.h>

#include <vector>

namespace tier4_autoware_utils::bg
{
// correct
void correct(tier4_autoware_utils::Polygon2d &);
void correct(tier4_autoware_utils::LineString2d &);

// within
bool within(const tier4_autoware_utils::Polygon2d &, const tier4_autoware_utils::Polygon2d &);
bool within(const tier4_autoware_utils::Point2d &, const tier4_autoware_utils::Polygon2d &);
bool within(const tier4_autoware_utils::Polygon2d &, const tier4_autoware_utils::LinearRing2d &);
bool within(const tier4_autoware_utils::Point2d &, const tier4_autoware_utils::LinearRing2d &);
bool within(const tier4_autoware_utils::Point2d &, const lanelet::BasicPolygon2d &);
bool within(const tier4_autoware_utils::Point2d &, const lanelet::CompoundPolygon2d &);

// convex_hull
void convex_hull(const tier4_autoware_utils::Polygon2d &, tier4_autoware_utils::Polygon2d &);
void convex_hull(const tier4_autoware_utils::MultiPoint2d &, tier4_autoware_utils::LinearRing2d &);

// intersects
bool intersects(
  const tier4_autoware_utils::Polygon2d &, const tier4_autoware_utils::LineString2d &);
bool intersects(const tier4_autoware_utils::Polygon2d &, const tier4_autoware_utils::Polygon2d &);
bool intersects(
  const tier4_autoware_utils::LineString2d &, const tier4_autoware_utils::LineString2d &);
bool intersects(const tier4_autoware_utils::LinearRing2d &, const lanelet::BasicPolygon2d &);

// intersection
void intersection(
  const tier4_autoware_utils::LineString2d &, const tier4_autoware_utils::LineString2d &,
  std::vector<tier4_autoware_utils::Point2d> &);
void intersection(
  const tier4_autoware_utils::Polygon2d &, const tier4_autoware_utils::LineString2d &,
  std::vector<tier4_autoware_utils::Point2d> &);
void intersection(
  const tier4_autoware_utils::Polygon2d &, const tier4_autoware_utils::Polygon2d &,
  std::vector<tier4_autoware_utils::Point2d> &);
void intersection(
  const tier4_autoware_utils::Polygon2d &, const lanelet::BasicPolygon2d &,
  std::vector<tier4_autoware_utils::Point2d> &);

// disjoint
bool disjoint(const tier4_autoware_utils::Polygon2d &, const tier4_autoware_utils::Polygon2d &);
}  // namespace tier4_autoware_utils::bg

#endif  // TIER4_AUTOWARE_UTILS__GEOMETRY__BOOST_GEOMETRY_ALGORITHMS_HPP_
