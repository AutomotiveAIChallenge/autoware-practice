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

#ifndef VISUALIZATION_HPP_
#define VISUALIZATION_HPP_

#include "geometry.hpp"

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace autoware_practice_evaluator
{

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

class RvizMarker
{
public:
  RvizMarker();
  Marker message() const;
  void polygon(const Polygon & polygon);
  void color(double r, double g, double b, double a = 1.0);

private:
  Marker marker_;
};

// polygon to triangle list

}  // namespace autoware_practice_evaluator

#endif  // VISUALIZATION_HPP_
