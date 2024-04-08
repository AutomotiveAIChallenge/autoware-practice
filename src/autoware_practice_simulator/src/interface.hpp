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

#ifndef INTERFACE_HPP_
#define INTERFACE_HPP_

#include "geometry.hpp"

#include <array>

namespace autoware_practice_simulator
{

struct VehicleSpecs
{
  std::array<double, 4> overhang;
  double height;
  double wheel_tread;
  double wheel_base;
  double mass;
  double max_speed;
  double max_accel;
  double max_brake;
  double max_steer;
};

struct VehicleState
{
  Point2 point;
  double speed;
  double angle;
};

}  // namespace autoware_practice_simulator

#endif  // INTERFACE_HPP_
