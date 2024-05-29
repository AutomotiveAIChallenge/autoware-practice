#ifndef TRAJECTORY_FOLLOWER_HPP_
#define TRAJECTORY_FOLLOWER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp> 
#include <geometry_msgs/msg/quaternion.hpp>

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
  using Point = geometry_msgs::msg::Point;
  using Quaternion = geometry_msgs::msg::Quaternion;

  void on_timer();
  void update_target_velocity(const Trajectory & msg);
  void update_current_state(const Odometry & msg);
  double longitudinal_controller(double veloctiy_error);
  double lateral_controller();
  double calculateYawFromQuaternion(const geometry_msgs::msg::Quaternion& q);
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_command_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_kinematic_state_;
  Trajectory trajectory_;
  Point current_position_;
  Quaternion current_orientation_;
  
  double current_velocity_;
  double target_velocity_;
  double kp_;
  double lookahead_distance_;
  double wheel_base_;
  size_t closest_point_index_;
};

} 

#endif