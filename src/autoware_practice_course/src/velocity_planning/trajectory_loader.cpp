#include "trajectory_loader.hpp"
#include <string>
#include <fstream>
#include <sstream>
#include <memory>
#include <cmath>

namespace autoware_practice_course
{

SampleNode::SampleNode() : Node("trajectory_loader")
{
  using std::placeholders::_1;
  pub_trajectory_ = create_publisher<Trajectory>("/planning/scenario_planning/trajectory", rclcpp::QoS(1));
  sub_kinematic_state_ = create_subscription<Odometry>("/localization/kinematic_state", rclcpp::QoS(1), std::bind(&SampleNode::update_vehicle_position, this, _1));
  
  this->declare_parameter<std::string>("path_file", "path.csv");
  auto path_file = this->get_parameter("path_file").as_string();
  load_path(path_file);
  RCLCPP_INFO(this->get_logger(), "complete loading trajectory");

  const auto period = rclcpp::Rate(10).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this] { on_timer(); });
}

void SampleNode::update_vehicle_position(const Odometry & msg)
{
  position_x_ = msg.pose.pose.position.x;
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


} 

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<autoware_practice_course::SampleNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
