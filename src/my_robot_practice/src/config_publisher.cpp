#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>

class ConfigPublisher : public rclcpp::Node
{
public:
    ConfigPublisher() : Node("config_publisher")
    {
        RCLCPP_INFO(this->get_logger(), "ConfigPublisher Node has been started.");
        auto qos = rclcpp::QoS(1).transient_local();  // Use best effort QoS for config messages
        // Create a publisher to the "config" topic
        publisher_ = this->create_publisher<std_msgs::msg::String>("config", qos);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                auto message = std_msgs::msg::String();
                message.data = "robot: navbot, max_speed: 0.5";
                RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
                publisher_->publish(message);
                timer_->cancel();   // stop after the first publish

            });
        
        // timer_->cancel();  // Start with the timer stopped
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ConfigPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}