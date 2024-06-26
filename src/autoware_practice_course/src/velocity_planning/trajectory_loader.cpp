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

#include "trajectory_loader.hpp"

#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>

namespace autoware_practice_course
{

SampleNode::SampleNode() : Node("trajectory_loader")
{
  pub_trajectory_ = create_publisher<Trajectory>("/planning/trajectory_loader/trajectory", rclcpp::QoS(1));
  this->declare_parameter<std::string>("path_file", "path.csv");
  auto path_file = this->get_parameter("path_file").as_string();
  load_path(path_file);

  const auto period = rclcpp::Rate(10).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this] { on_timer(); });
}

void SampleNode::on_timer()
{
  pub_trajectory_->publish(trajectory_);
}

void SampleNode::load_path(const std::string & file_path)
{
  std::ifstream file(file_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_path.c_str());
    return;
  }
  std::string line;
  std::getline(file, line);
  RCLCPP_INFO(this->get_logger(), "Skipping header: %s", line.c_str());
  while (std::getline(file, line)) {
    RCLCPP_INFO(this->get_logger(), "Processing line: %s", line.c_str());
    TrajectoryPoint point;
    std::stringstream ss(line);
    std::string x, y, longitudinal_velocity_mps;
    std::getline(ss, x, ',');
    std::getline(ss, y, ',');
    std::getline(ss, longitudinal_velocity_mps, ',');

    point.pose.position.x = std::stod(x);
    point.pose.position.y = std::stod(y);
    point.longitudinal_velocity_mps = std::stod(longitudinal_velocity_mps);
    trajectory_.points.push_back(point);
  }
  file.close();
}

}  // namespace autoware_practice_course

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<autoware_practice_course::SampleNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
