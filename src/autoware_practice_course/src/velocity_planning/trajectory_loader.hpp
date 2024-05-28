#ifndef TRAJECTORY_LOADER_HPP_
#define TRAJECTORY_LOADER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <nav_msgs/msg/odometry.hpp>


namespace autoware_practice_course
{

class SampleNode : public rclcpp::Node
{
public:
  SampleNode();

private:
  using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
  using TrajectoryPoint = autoware_auto_planning_msgs::msg::TrajectoryPoint;
  using Odometry = nav_msgs::msg::Odometry;

  void on_timer();
  void update_vehicle_position(const Odometry & msg);
  void load_path(const std::string & file_path);
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_trajectory_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_kinematic_state_;
  Trajectory trajectory_;
  double position_x_;

};

} 

#endif
