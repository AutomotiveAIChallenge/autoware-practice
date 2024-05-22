#ifndef TRAJECTORY_PLANNER_HPP_
#define TRAJECTORY_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <nav_msgs/msg/odometry.hpp>
namespace autoware_practice_course
{

class SampleNode : public rclcpp::Node
{
public:
  SampleNode();

private:
  using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
  using AckermannControlCommand = autoware_auto_control_msgs::msg::AckermannControlCommand;
  using Odometry = nav_msgs::msg::Odometry;
  using Pose = geometry_msgs::msg::Pose;

  void on_timer();
  void update_target_velocity(const Trajectory & msg);
  void update_current_state(const Odometry & msg);
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_command_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_kinematic_state_;
  double current_pose_;
  double current_velocity_;
  double target_velocity_;
  double kp_;
};

} 

#endif
