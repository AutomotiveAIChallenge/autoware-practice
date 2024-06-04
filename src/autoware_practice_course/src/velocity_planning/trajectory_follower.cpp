#include "trajectory_follower.hpp"
#include <fstream>

namespace autoware_practice_course
{

SampleNode::SampleNode() : Node("trajectory_follower"), kp_(0.0), lookahead_distance_(5.0)
{
  using std::placeholders::_1;
  declare_parameter<double>("kp", kp_);
  get_parameter("kp", kp_);
  declare_parameter<double>("lookahead_distance", lookahead_distance_);
  get_parameter("lookahead_distance", lookahead_distance_);
  param_file_ = "./src/autoware_practice_simulator/config/vehicle.param.yaml";
  wheel_base_ = load_parameters(param_file_, "wheel_base");

  RCLCPP_INFO(this->get_logger(), "Wheel base: %f", wheel_base_);

  pub_command_ = create_publisher<AckermannControlCommand>("/control/command/control_cmd", rclcpp::QoS(1));
  sub_trajectory_ = create_subscription<Trajectory>("/planning/scenario_planning/trajectory", rclcpp::QoS(1), std::bind(&SampleNode::update_target_velocity, this, _1));
  sub_kinematic_state_= create_subscription<Odometry>("/localization/kinematic_state", rclcpp::QoS(1), std::bind(&SampleNode::update_current_state, this, _1));

  const auto period = rclcpp::Rate(10).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this] { on_timer(); });
}

void SampleNode::update_target_velocity(const Trajectory & msg)
{
  double min_distance = std::numeric_limits<double>::max();
  

  for (size_t i = 0; i < msg.points.size(); ++i) {
    double dx = msg.points[i].pose.position.x - current_position_.x;
    double dy = msg.points[i].pose.position.y - current_position_.y;
    double dz = msg.points[i].pose.position.z - current_position_.z;
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (distance < min_distance) {
      min_distance = distance;
      closest_point_index_ = i;
    }
  }

  trajectory_ = msg;
  target_velocity_ = msg.points[closest_point_index_].longitudinal_velocity_mps;
};

double SampleNode::loadParameters(const std::string & param_file, const std::string & param_tag)
{
  std::ifstream file(param_file);
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open parameter file: %s", param_file.c_str());
    return -1.0;
  }

  std::string line;
  while (std::getline(file, line)) {
    if (line.find(param_tag + ":") != std::string::npos) {
      size_t pos = line.find(":");
      if (pos != std::string::npos) {
        double param_value = std::stod(line.substr(pos + 1));
        return param_value;
      }
    }
  }

  RCLCPP_ERROR(this->get_logger(), "Parameter %s not found in file: %s", param_tag.c_str(), param_file.c_str());
  return -1.0;
}


void SampleNode::update_current_state(const Odometry & msg)
{
  current_velocity_ = msg.twist.twist.linear.x;
  current_position_ = msg.pose.pose.position;
  current_orientation_ = msg.pose.pose.orientation;
};

void SampleNode::on_timer()
{
  const auto stamp = now();

  AckermannControlCommand command;
  command.stamp = stamp;
  
  double velocity_error = target_velocity_ - current_velocity_;
  command.longitudinal.acceleration = longitudinal_controller(velocity_error);
  command.longitudinal.speed = target_velocity_;
  
  command.lateral.steering_tire_angle = lateral_controller();
  
  pub_command_->publish(command);
}

double SampleNode::longitudinal_controller(double velocity_error)
{
  return kp_ * velocity_error;
}

double SampleNode::lateral_controller()
{
  double min_distance = std::numeric_limits<double>::max();
  size_t lookahead_point_index = closest_point_index_;
  min_distance = std::numeric_limits<double>::max();
  // Find the lookahead point
  for (size_t i = closest_point_index_; i < trajectory_.points.size(); ++i) {
    double dx = trajectory_.points[i].pose.position.x - current_position_.x;
    double dy = trajectory_.points[i].pose.position.y - current_position_.y;
    double dz = trajectory_.points[i].pose.position.z - current_position_.z;
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (distance >= lookahead_distance_ && distance < min_distance) {
      min_distance = distance;
      lookahead_point_index = i;
    }
  }
  if (lookahead_point_index == closest_point_index_) {
    RCLCPP_WARN(this->get_logger(), "No valid lookahead point found.");
    return 0.0;
  }
  double dx = trajectory_.points[lookahead_point_index].pose.position.x - current_position_.x;
  double dy = trajectory_.points[lookahead_point_index].pose.position.y - current_position_.y;
  double alpha = std::atan2(dy, dx) - calculate_yaw_from_quaternion(current_orientation_);
  double steering_angle = std::atan2(2.0 * wheel_base_ * std::sin(alpha), lookahead_distance_);
  RCLCPP_INFO(this->get_logger(), "Calculated steering angle: %f", steering_angle);
  return steering_angle;
}

double SampleNode::calculate_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q) {
    // Convert quaternion to Euler angles
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    return yaw;
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