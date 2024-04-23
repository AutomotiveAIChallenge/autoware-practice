#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "nav_msgs/msg/odometry.hpp"

class Localizer : public rclcpp::Node
{
public:
    Localizer() : Node("localizer")
    {
        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/localization/kinematic_state", 10);
        pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovariance>(
            "/sensing/gnss/pose_with_covariance", 10, std::bind(&Localizer::pose_callback, this, std::placeholders::_1));
        twist_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistWithCovariance>(
            "/localization/twist_estimator/twist_with_covariance", 10, std::bind(&Localizer::twist_callback, this, std::placeholders::_1));
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseWithCovariance::SharedPtr msg)
    {
        last_pose_ = *msg;
        publish_odometry();
    }

    void twist_callback(const geometry_msgs::msg::TwistWithCovariance::SharedPtr msg)
    {
        last_twist_ = *msg;
        publish_odometry();
    }

    void publish_odometry()
    {
        if (last_pose_ && last_twist_) {
            nav_msgs::msg::Odometry odom;
            odom.header.stamp = this->get_clock()->now();
            odom.header.frame_id = "odom";
            odom.child_frame_id = "base_link";
            odom.pose = *last_pose_;
            odom.twist = *last_twist_;
            publisher_->publish(odom);
            last_pose_.reset();
            last_twist_.reset();
        }
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovariance>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovariance>::SharedPtr twist_subscriber_;
    std::optional<geometry_msgs::msg::PoseWithCovariance> last_pose_;
    std::optional<geometry_msgs::msg::TwistWithCovariance> last_twist_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Localizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
