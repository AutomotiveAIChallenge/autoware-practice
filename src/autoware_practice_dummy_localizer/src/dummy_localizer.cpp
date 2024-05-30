#include "dummy_localizer.hpp"



namespace autoware_practice_course
{
SampleNode::SampleNode() : Node("dummy_localizer"), has_pose_(false), has_twist_(false)
{
    publisher_ = create_publisher<Odometry>("/localization/kinematic_state", 10);
    pose_subscriber_ = create_subscription<PoseStamped>(
        "/simulator/ground_truth/pose", 10, std::bind(&SampleNode::pose_callback, this, std::placeholders::_1));
    twist_subscriber_ = create_subscription<TwistStamped>(
        "/simulator/ground_truth/twist", 10, std::bind(&SampleNode::twist_callback, this, std::placeholders::_1));
}

void SampleNode::pose_callback(const PoseStamped::SharedPtr msg)
{
    last_pose_ = msg->pose;
    has_pose_ = true;
    publish_odometry();
}

void SampleNode::twist_callback(const TwistStamped::SharedPtr msg)
{
    last_twist_ = msg->twist;
    has_twist_ = true;
    publish_odometry();
}

void SampleNode::publish_odometry()
{
    if (has_pose_ && has_twist_) {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose = last_pose_;
        odom.twist.twist = last_twist_;
        publisher_->publish(odom);
        has_pose_ = false;
        has_twist_ = false;
    }
}

}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<autoware_practice_course::SampleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
