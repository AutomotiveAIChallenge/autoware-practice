// Copyright 2024 TIER IV, Inc.
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

#ifndef GEOMETRY_HPP_
#define GEOMETRY_HPP_

#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/ring.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <vector>

namespace autoware_practice_evaluator
{

using Point = geometry_msgs::msg::Point;
using Polygon = std::vector<Point>;

Point Point2(double x, double y);

}  // namespace autoware_practice_evaluator

BOOST_GEOMETRY_REGISTER_POINT_3D(autoware_practice_evaluator::Point, double, boost::geometry::cs::cartesian, x, y, z)  // NOLINT
BOOST_GEOMETRY_REGISTER_RING(autoware_practice_evaluator::Polygon)                                                     // NOLINT

#endif  // GEOMETRY_HPP_
