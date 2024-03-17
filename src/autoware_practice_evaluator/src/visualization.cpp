// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under #include <memory>the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "visualization.hpp"

namespace autoware_practice_evaluator
{

RvizMarker::RvizMarker()
{
  marker_.header.frame_id = "map";
  marker_.ns = "";
  marker_.id = 0;
  marker_.action = Marker::ADD;
  marker_.pose.orientation.w = 1.0;
  marker_.scale.x = 1.0;
  marker_.scale.y = 1.0;
  marker_.scale.z = 1.0;
}

Marker RvizMarker::message() const
{
  return marker_;
}

void RvizMarker::polygon(const Polygon & polygon)
{
  // NOTE: Convex polygon only.
  marker_.type = Marker::TRIANGLE_LIST;
  for (size_t i = 2; i < polygon.size(); ++i) {
    marker_.points.push_back(polygon[0]);
    marker_.points.push_back(polygon[i - 1]);
    marker_.points.push_back(polygon[i]);
  }
}

void RvizMarker::color(double r, double g, double b, double a)
{
  marker_.color.r = r;
  marker_.color.g = g;
  marker_.color.b = b;
  marker_.color.a = a;
}

}  // namespace autoware_practice_evaluator
