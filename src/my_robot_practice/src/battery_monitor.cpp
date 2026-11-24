#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/battery_status.hpp"
#include <memory>
#include <string>

class BatteryMonitorNode : public rclcpp::Node
{
public:
    BatteryMonitorNode() : Node("battery_monitor_node")
    {
        RCLCPP_INFO(this->get_logger(), "BatteryMonitorNode has been started.");
        // Create a subscription to the "battery_status" topic
        subscription_ = this->create_subscription<my_robot_interfaces::msg::BatteryStatus>(
            "battery_status", 10,
            [this](my_robot_interfaces::msg::BatteryStatus::UniquePtr msg) {
                // Output like: Battery LOW: 10.9 V (20.0%)
                if (msg->status == last_status_) {
                    return;            // same as before, so don't log
                }
                last_status_ = msg->status;
                const char * status = msg->status.c_str();
                if (msg->status == "OK") {
                    RCLCPP_INFO(this->get_logger(), "Battery %s: %.1f V (%.1f%%)",
                                status, msg->voltage, msg->percentage);
                } else if (msg->status == "LOW") {
                    RCLCPP_WARN(this->get_logger(), "Battery %s: %.1f V (%.1f%%)",
                                status, msg->voltage, msg->percentage);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Battery %s: %.1f V (%.1f%%)",
                                 status, msg->voltage, msg->percentage);
                }
            });
    }   

    private:
    rclcpp::Subscription<my_robot_interfaces::msg::BatteryStatus>::SharedPtr subscription_;
    std::string last_status_;
};
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryMonitorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}