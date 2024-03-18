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

#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace autoware_practice_simulator
{

using geometry_msgs::msg::AccelStamped;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Quaternion;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::TwistStamped;

constexpr double pi = 3.141592653589793;

struct Point2
{
  Point2 rotate(double angle) const;
  Point2 & operator+=(const Point2 & p);
  Point2 & operator-=(const Point2 & p);

  double x;
  double y;
};

Point2 operator+(const Point2 & p1, const Point2 & p2);
Point2 operator-(const Point2 & p1, const Point2 & p2);

Quaternion yaw_to_quaternion(double yaw);

}  // namespace autoware_practice_simulator

#endif  // GEOMETRY_HPP_
