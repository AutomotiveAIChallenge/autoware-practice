#ifndef P_CONTROLLER_HPP_
#define P_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>

namespace autoware_practice_course
{

class SampleNode : public rclcpp::Node
{
public:
  SampleNode();

private:
  using AckermannControlCommand = autoware_auto_control_msgs::msg::AckermannControlCommand;
  using VelocityReport = autoware_auto_vehicle_msgs::msg::VelocityReport;

  void on_timer();

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_command_;

  rclcpp::Subscription<VelocityReport>::SharedPtr velocity_subscriber_;

  double current_velocity_;

  double target_velocity_;

  double kp_;

  void velocity_callback(const VelocityReport::SharedPtr msg);
};

} 

#endif
