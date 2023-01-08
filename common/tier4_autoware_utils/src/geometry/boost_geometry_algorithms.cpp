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

#include <tier4_autoware_utils/geometry/boost_geometry_algorithms.hpp>

#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/geometry/algorithms/is_empty.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/strategies/strategies.hpp>

#include <lanelet2_core/geometry/Polygon.h>

namespace tier4_autoware_utils::bg
{
// correct
void correct(tier4_autoware_utils::Polygon2d & polygon) { boost::geometry::correct(polygon); }

// within
bool within(
  const tier4_autoware_utils::Polygon2d & poly1, const tier4_autoware_utils::Polygon2d & poly2)
{
  return boost::geometry::within(poly1, poly2);
}
bool within(
  const tier4_autoware_utils::Point2d & poly1, const tier4_autoware_utils::Polygon2d & poly2)
{
  return boost::geometry::within(poly1, poly2);
}
bool within(
  const tier4_autoware_utils::Polygon2d & poly1, const tier4_autoware_utils::LinearRing2d & poly2)
{
  return boost::geometry::within(poly1, poly2);
}
bool within(
  const tier4_autoware_utils::Point2d & poly1, const tier4_autoware_utils::LinearRing2d & poly2)
{
  return boost::geometry::within(poly1, poly2);
}
bool within(const tier4_autoware_utils::Point2d & poly1, const lanelet::BasicPolygon2d & poly2)
{
  return boost::geometry::within(poly1, poly2);
}
bool within(const tier4_autoware_utils::Point2d & poly1, const lanelet::CompoundPolygon2d & poly2)
{
  return boost::geometry::within(poly1, poly2);
}

// convex_hull
void convex_hull(
  const tier4_autoware_utils::Polygon2d & poly1, tier4_autoware_utils::Polygon2d & poly2)
{
  boost::geometry::convex_hull(poly1, poly2);
}
void convex_hull(
  const tier4_autoware_utils::MultiPoint2d & poly1, tier4_autoware_utils::LinearRing2d & poly2)
{
  boost::geometry::convex_hull(poly1, poly2);
}

// intersects
bool intersects(
  const tier4_autoware_utils::Polygon2d & poly1, const tier4_autoware_utils::LineString2d & poly2)
{
  return boost::geometry::intersects(poly1, poly2);
}
bool intersects(
  const tier4_autoware_utils::Polygon2d & poly1, const tier4_autoware_utils::Polygon2d & poly2)
{
  return boost::geometry::intersects(poly1, poly2);
}
bool intersects(
  const tier4_autoware_utils::LineString2d & poly1,
  const tier4_autoware_utils::LineString2d & poly2)
{
  return boost::geometry::intersects(poly1, poly2);
}

// intersection
void intersection(
  const tier4_autoware_utils::LineString2d & poly1,
  const tier4_autoware_utils::LineString2d & poly2,
  std::vector<tier4_autoware_utils::Point2d> & poly3)
{
  boost::geometry::intersection(poly1, poly2, poly3);
}
void intersection(
  const tier4_autoware_utils::Polygon2d & poly1, const tier4_autoware_utils::LineString2d & poly2,
  std::vector<tier4_autoware_utils::Point2d> & poly3)
{
  boost::geometry::intersection(poly1, poly2, poly3);
}

// disjoint
bool disjoint(
  const tier4_autoware_utils::Polygon2d & poly1, const tier4_autoware_utils::Polygon2d & poly2)
{
  return boost::geometry::disjoint(poly1, poly2);
}

}  // namespace tier4_autoware_utils::bg
