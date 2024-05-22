#include "longitudinal_controller.hpp"

#include <memory>

namespace autoware_practice_course
{

SampleNode::SampleNode() : Node("longitudinal_controller"), kp_(0.0)
{
  using std::placeholders::_1;
  declare_parameter<double>("kp", kp_);
  get_parameter("kp", kp_);

  pub_command_ = create_publisher<AckermannControlCommand>("/control/command/control_cmd", rclcpp::QoS(1));
  sub_trajectory_ = create_subscription<Trajectory>("/planning/scenario_planning/trajectory", rclcpp::QoS(1), std::bind(&SampleNode::update_target_velocity, this, _1));
  sub_kinematic_state_= create_subscription<Odometry>("/localization/kinematic_state", rclcpp::QoS(1), std::bind(&SampleNode::update_current_state, this, _1));

  const auto period = rclcpp::Rate(10).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this] { on_timer(); });
}

void SampleNode::update_target_velocity(const Trajectory & msg)
{
  double min_distance = std::numeric_limits<double>::max();
  size_t closest_waypoint_index = 0;

  for (size_t i = 0; i < msg.points.size(); ++i) {
    double dx = msg.points[i].pose.position.x - current_pose_;
    double distance = std::abs(dx);

    if (distance < min_distance) {
      min_distance = distance;
      closest_waypoint_index = i;
    }
  }

  target_velocity_ = msg.points[closest_waypoint_index].longitudinal_velocity_mps;
};

void SampleNode::update_current_state(const Odometry & msg)
{
  current_velocity_ = msg.twist.twist.linear.x;
  current_pose_ = msg.pose.pose.position.x;  // 現在の車両の位置を更新する
};

void SampleNode::on_timer()
{
  const auto stamp = now();

  AckermannControlCommand command;
  command.stamp = stamp;
  
  double velocity_error = target_velocity_ - current_velocity_;
  command.longitudinal.acceleration = kp_ * velocity_error;
  command.longitudinal.speed = target_velocity_; // メッセージ型としてはspeedがあるが、vehiclle interface側では加速度しか受け取っていない。
  
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
