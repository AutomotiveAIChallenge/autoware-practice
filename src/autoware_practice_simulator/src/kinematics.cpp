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

#include "kinematics.hpp"

#include <algorithm>
#include <cmath>

namespace autoware_practice_simulator
{

VehicleKinematics::VehicleKinematics()
{
  state_.point = Point2{0.0, 0.0};
  state_.speed = 0.0;
  state_.angle = 0.0;
}

void VehicleKinematics::update(double dt, const ArcPath & input)
{
  Point2 point_delta;
  double angle_delta;

  if (input.radius == 0.0) {
    angle_delta = 0.0;
    point_delta = Point2{input.speed * dt, 0.0};
  } else {
    angle_delta = input.speed * dt / input.radius;
    point_delta = Point2{input.radius, 0.0}.rotate(angle_delta);
    point_delta = Point2{point_delta.y, input.radius - point_delta.x};
  }

  state_.angle += angle_delta;
  state_.point += point_delta.rotate(state_.angle);
  state_.speed = input.speed;
}

}  // namespace autoware_practice_simulator
