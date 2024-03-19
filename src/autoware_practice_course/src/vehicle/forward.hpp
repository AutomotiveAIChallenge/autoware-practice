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

#ifndef VEHICLE__FORWARD_HPP_
#define VEHICLE__FORWARD_HPP_

#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>

namespace autoware_practice_course
{

class SampleNode : public rclcpp::Node
{
public:
  SampleNode();

private:
  using AckermannControlCommand = autoware_auto_control_msgs::msg::AckermannControlCommand;
  void on_timer();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_command_;
};

}  // namespace autoware_practice_course

#endif  // VEHICLE__FORWARD_HPP_
