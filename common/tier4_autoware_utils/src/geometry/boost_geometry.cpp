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

#include <tier4_autoware_utils/geometry/boost_geometry.hpp>

namespace boost::geometry
{
// correct
template void correct(tier4_autoware_utils::Polygon2d &);
// within
template bool within(
  const tier4_autoware_utils::Polygon2d &, const tier4_autoware_utils::Polygon2d &);
template bool within(
  const tier4_autoware_utils::Point2d &, const tier4_autoware_utils::Polygon2d &);
template bool within(
  const tier4_autoware_utils::Polygon2d &, const tier4_autoware_utils::LinearRing2d &);
template bool within(
  const tier4_autoware_utils::Point2d &, const tier4_autoware_utils::LinearRing2d &);
// is_empty
template bool is_empty(const tier4_autoware_utils::Polygon2d &);
// convex_hull
template void convex_hull(
  const tier4_autoware_utils::Polygon2d &, tier4_autoware_utils::Polygon2d &);
template void convex_hull(
  const tier4_autoware_utils::MultiPoint2d &, tier4_autoware_utils::LinearRing2d &);
// intersects
template bool intersects(
  const tier4_autoware_utils::Polygon2d &, const tier4_autoware_utils::LineString2d &);
template bool intersects(
  const tier4_autoware_utils::Polygon2d &, const tier4_autoware_utils::Polygon2d &);
template bool intersects(
  const tier4_autoware_utils::LineString2d &, const tier4_autoware_utils::LineString2d &);

}  // namespace boost::geometry
