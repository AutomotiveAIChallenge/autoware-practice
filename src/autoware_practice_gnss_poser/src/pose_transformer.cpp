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

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"

class PoseTransformer : public rclcpp::Node
{
public:
  PoseTransformer() : Node("pose_transformer")
  {
    publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovariance>("/sensing/gnss/pose_with_covariance", 10);
    subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/simulator/ground_truth/pose", 10, std::bind(&PoseTransformer::pose_callback, this, std::placeholders::_1));
  }

private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    geometry_msgs::msg::PoseWithCovariance new_msg;
    new_msg.pose = msg->pose;  // Directly assign the Pose part

    // Set a dummy covariance matrix
    for (int i = 0; i < 36; ++i) {
      new_msg.covariance[i] = 0.0;  // Set all covariance to 0.0 for simplicity
    }

    // Set some values to covariance for demonstration
    new_msg.covariance[0] = 0.1;    // Variance in x
    new_msg.covariance[7] = 0.1;    // Variance in y
    new_msg.covariance[14] = 0.1;   // Variance in z
    new_msg.covariance[21] = 0.01;  // Variance in rotation around X
    new_msg.covariance[28] = 0.01;  // Variance in rotation around Y
    new_msg.covariance[35] = 0.01;  // Variance in rotation around Z

    publisher_->publish(new_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovariance>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseTransformer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
