#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"

class VelocityTransformer : public rclcpp::Node
{
public:
    VelocityTransformer() : Node("velocity_transformer")
    {
        publisher_ = create_publisher<geometry_msgs::msg::TwistWithCovariance>("/localization/twist_estimator/twist_with_covariance", 10);
        subscription_ = create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>(
            "/vehicle/status/velocity_status", 10, std::bind(&VelocityTransformer::velocity_callback, this, std::placeholders::_1));
    }

private:
    void velocity_callback(const autoware_auto_vehicle_msgs::msg::VelocityReport::SharedPtr msg)
    {
        geometry_msgs::msg::TwistWithCovariance twist_msg;
        twist_msg.twist.linear.x = msg->longitudinal_velocity;
        twist_msg.twist.linear.y = msg->lateral_velocity;
        twist_msg.twist.angular.z = msg->heading_rate;

        // Initialize covariance as zero for simplicity, just an example
        for (int i = 0; i < 36; ++i) {
            twist_msg.covariance[i] = 0.0;
        }
        // Setting specific covariance values for demonstration
        twist_msg.covariance[0] = 0.01; // Variance of longitudinal velocity
        twist_msg.covariance[7] = 0.01; // Variance of lateral velocity
        twist_msg.covariance[35] = 0.01; // Variance of heading rate

        publisher_->publish(twist_msg);
    }


    rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovariance>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VelocityTransformer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
