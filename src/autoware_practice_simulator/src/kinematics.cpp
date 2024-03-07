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

Kinematics::Kinematics(const VehicleSpecs & specs)
{
  specs_ = specs;

  input_.accel = 0.0;
  input_.steer = 0.0;
  state_.gear = Gear::Drive;

  state_.point = Point2{0.0, 0.0};
  state_.speed = 0.0;
  state_.accel = 0.0;
  state_.brake = 0.0;
  state_.angle = 0.0;
  state_.steer = 0.0;
}

VehicleSpecs Kinematics::specs() const
{
  return specs_;
}

Gear Kinematics::gear() const
{
  return state_.gear;
}

Pose Kinematics::pose() const
{
  Pose pose;
  pose.position.x = state_.point.x;
  pose.position.y = state_.point.y;
  pose.orientation = yaw_to_quaternion(state_.angle);
  return pose;
}

double Kinematics::speed() const
{
  return state_.speed;
}

double Kinematics::steer() const
{
  return state_.steer;
}

void Kinematics::update_input(const VehicleInput & input)
{
  input_ = input;
}

void Kinematics::update_gear(const Gear & gear)
{
  state_.gear = gear;
}

void Kinematics::update_state(double dt)
{
  switch (state_.gear) {
    case Gear::Parking:
      state_.accel = 0.0;
      state_.brake = specs_.max_brake;
      break;
    case Gear::Neutral:
      state_.accel = 0.0;
      state_.brake = 0.0;
      break;
    case Gear::Drive:
      state_.accel = std::max(+input_.accel, 0.0);
      state_.brake = std::max(-input_.accel, 0.0);
      break;
    case Gear::Reverse:
      state_.accel = std::max(+input_.accel, 0.0) * (-1.0);
      state_.brake = std::max(-input_.accel, 0.0);
      break;
  }
  state_.steer = input_.steer;

  // brake max_brake
  state_.accel = std::clamp(state_.accel, -specs_.max_accel, +specs_.max_accel);
  state_.brake = std::clamp(state_.brake, -specs_.max_brake, +specs_.max_brake);
  state_.steer = std::clamp(state_.steer, -specs_.max_steer, +specs_.max_steer);

  const auto sign = std::signbit(state_.speed) ? -1.0 : +1.0;
  state_.speed -= std::min(state_.brake * dt, std::abs(state_.speed)) * sign;
  state_.speed += state_.accel * dt;
  state_.speed = std::clamp(state_.speed, -specs_.max_speed, +specs_.max_speed);

  update_point(dt);
}

void Kinematics::update_point(double dt)
{
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
