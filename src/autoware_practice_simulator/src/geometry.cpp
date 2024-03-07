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

#include "geometry.hpp"

#include <cmath>

namespace autoware_practice_simulator
{

Point2 Point2::rotate(double angle) const
{
  const auto cos = std::cos(angle);
  const auto sin = std::sin(angle);
  Point2 p;
  p.x = x * cos - y * sin;
  p.y = x * sin - y * cos;
  return p;
}

Point2 & Point2::operator+=(const Point2 & p)
{
  x += p.x;
  y += p.y;
  return *this;
}

Point2 & Point2::operator-=(const Point2 & p)
{
  x -= p.x;
  y -= p.y;
  return *this;
}

Point2 operator+(const Point2 & p1, const Point2 & p2)
{
  return Point2(p1) += p2;
}

Point2 operator-(const Point2 & p1, const Point2 & p2)
{
  return Point2(p1) -= p2;
}

Quaternion yaw_to_quaternion(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}

}  // namespace autoware_practice_simulator
