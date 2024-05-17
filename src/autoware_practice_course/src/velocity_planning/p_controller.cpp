#include "p_controller.hpp"

#include <memory>

namespace autoware_practice_course
{

SampleNode::SampleNode() : Node("p_controller")
{
  declare_parameter<double>("kp", 0.0);
  declare_parameter<double>("target_velocity", 1.0);

  get_parameter("kp", kp_);
  get_parameter("target_velocity", target_velocity_);

  pub_command_ = create_publisher<AckermannControlCommand>("/control/command/control_cmd", rclcpp::QoS(1));
  
  velocity_subscriber_ = create_subscription<VelocityReport>(
    "/vehicle/status/velocity_status", 10, [this](const VelocityReport::SharedPtr msg) {
      current_velocity_ = msg->longitudinal_velocity; 
  });

  const auto period = rclcpp::Rate(10).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this] { on_timer(); });
}

void SampleNode::on_timer()
{
  const auto stamp = now();

  AckermannControlCommand command;
  command.stamp = stamp;
  
  double velocity_error = target_velocity_ - current_velocity_;
  command.longitudinal.acceleration = kp_ * velocity_error;
  command.longitudinal.speed = target_velocity_;
  
  command.lateral.steering_tire_angle = 0.0;
  
  pub_command_->publish(command);
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
