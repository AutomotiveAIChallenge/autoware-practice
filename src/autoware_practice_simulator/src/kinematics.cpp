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

VehicleKinematics::VehicleKinematics(const VehicleSpecs & specs)
{
  specs_ = specs;

  state_.point = Point2{0.0, 0.0};
  state_.speed = 0.0;
  state_.accel = 0.0;
  state_.angle = 0.0;
  state_.steer = 0.0;
}

VehicleSpecs VehicleKinematics::specs() const
{
  return specs_;
}

VehicleState VehicleKinematics::state() const
{
  return state_;
}

void VehicleKinematics::update(double dt, const VehicleInput & input)
{
  const auto accel = std::clamp(input.accel, -specs_.max_accel, +specs_.max_accel);
  const auto steer = std::clamp(input.steer, -specs_.max_steer, +specs_.max_steer);

  const auto brake_sign = std::signbit(state_.speed) ? +1.0 : -1.0;
  const auto brake_limit = std::min(specs_.max_brake, std::abs(state_.speed) / dt);
  const auto brake = std::clamp(input.brake, 0.0, brake_limit) * brake_sign;

  state_.accel = accel + brake;
  state_.steer = steer;
  state_.speed = state_.speed + (state_.accel * dt);
  state_.speed = std::clamp(state_.speed, -specs_.max_speed, +specs_.max_speed);

  constexpr double steer_eps = 1e-3;
  double angle_delta;
  Point2 point_delta;

  if (std::abs(state_.steer) < steer_eps) {
    angle_delta = 0.0;
    point_delta = Point2{state_.speed * dt, 0.0};
  } else {
    const auto radius = specs_.wheel_base / std::tan(state_.steer);
    angle_delta = state_.speed * dt / radius;
    point_delta = Point2{radius, 0.0}.rotate(angle_delta);
    point_delta = Point2{point_delta.y, radius - point_delta.x};
  }

  state_.angle += angle_delta;
  state_.point += point_delta.rotate(state_.angle);
}

}  // namespace autoware_practice_simulator
