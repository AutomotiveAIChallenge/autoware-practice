#include "longitudinal_controller.hpp"
#include <string>
#include <fstream>
#include <sstream>

namespace autoware_practice_course
{

SampleNode::SampleNode() : Node("longitudinal_controller"), kp_(0.0)
{
  using std::placeholders::_1;
  declare_parameter<double>("kp", kp_);
  get_parameter("kp", kp_);
  pub_command_ = create_publisher<AckermannControlCommand>("/control/command/control_cmd", rclcpp::QoS(1));
  sub_kinematic_state_= create_subscription<Odometry>("/localization/kinematic_state", rclcpp::QoS(1), std::bind(&SampleNode::update_current_state, this, _1));
  this->declare_parameter<std::string>("path_file", "path.csv"); // パラメータの宣言
  auto path_file = this->get_parameter("path_file").as_string(); // パラメータの取得
  load_path(path_file); // パスの読み込み
  const auto period = rclcpp::Rate(10).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this] { on_timer(); });
  // on_timer()が一定間隔で呼ばれる。
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
        std::string x, y, z, orientation_x, orientation_y, orientation_z, orientation_w, longitudinal_velocity_mps, lateral_velocity_mps, acceleration_mps2, heading_rate_rps, front_wheel_angle_rad, rear_wheel_angle_rad;
        std::getline(ss, x, ',');
        std::getline(ss, y, ',');
        std::getline(ss, z, ',');
        std::getline(ss, orientation_x, ',');
        std::getline(ss, orientation_y, ',');
        std::getline(ss, orientation_z, ',');
        std::getline(ss, orientation_w, ',');
        std::getline(ss, longitudinal_velocity_mps, ',');
        std::getline(ss, lateral_velocity_mps, ',');
        std::getline(ss, acceleration_mps2, ',');
        std::getline(ss, heading_rate_rps, ',');
        std::getline(ss, front_wheel_angle_rad, ',');
        std::getline(ss, rear_wheel_angle_rad, ',');


        point.pose.position.x = std::stod(x);
        point.pose.position.y = std::stod(y);
        point.pose.position.z = std::stod(z);
        point.pose.orientation.x = std::stod(orientation_x);
        point.pose.orientation.y = std::stod(orientation_y);
        point.pose.orientation.z = std::stod(orientation_z);
        point.pose.orientation.w = std::stod(orientation_w);
        point.longitudinal_velocity_mps = std::stod(longitudinal_velocity_mps);
        point.lateral_velocity_mps = std::stod(lateral_velocity_mps);
        point.acceleration_mps2 = std::stod(acceleration_mps2);
        point.heading_rate_rps = std::stod(heading_rate_rps);
        point.front_wheel_angle_rad = std::stod(front_wheel_angle_rad);
        point.rear_wheel_angle_rad = std::stod(rear_wheel_angle_rad);
        trajectory_.points.push_back(point);
    }
    file.close();
}


void SampleNode::update_target_velocity(const Trajectory & msg)
{
  double min_distance = std::numeric_limits<double>::max();
  size_t closest_waypoint_index = 0;

  for (size_t i = 0; i < msg.points.size(); ++i) {
    double dx = msg.points[i].pose.position.x - current_pose_.x;
    double dy = msg.points[i].pose.position.y - current_pose_.y;
    double dz = msg.points[i].pose.position.z - current_pose_.z;
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (distance < min_distance) {
      min_distance = distance;
      closest_waypoint_index = i;
    }
  }
  target_velocity_ = msg.points[closest_waypoint_index].longitudinal_velocity_mps;
  RCLCPP_INFO(this->get_logger(), "Processing line: %f", target_velocity_);
};

void SampleNode::update_current_state(const Odometry & msg)
{
  current_velocity_ = msg.twist.twist.linear.x;
  current_pose_ = msg.pose.pose.position;  // 現在の車両の位置を更新する
};

void SampleNode::on_timer()
{
  
  update_target_velocity(trajectory_); // 目標速度を更新
  const auto stamp = now();
  AckermannControlCommand command;
  command.stamp = stamp;
  double velocity_error = target_velocity_ - current_velocity_;
  RCLCPP_INFO(this->get_logger(), "velocity error: %f", velocity_error);
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
