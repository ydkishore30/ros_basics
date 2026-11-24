#include "my_robot_interfaces/msg/battery_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <algorithm>
#include "std_srvs/srv/trigger.hpp"
#include <chrono>
#include <string>
#include <thread>




class BatteryNode : public rclcpp::Node
{
public:
    BatteryNode() : Node("battery_node")
    {

        RCLCPP_INFO(this->get_logger(), "BatteryNode has been started.");
            reset_srv_ = create_service<std_srvs::srv::Trigger>(
        "reset_battery",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            (void)request;   // not used, since Trigger has no request fields
            // your code: reset the battery state, fill in response->success and response->message
                            std::this_thread::sleep_for(std::chrono::seconds(3));   // #include <thread>

            voltage_ = kFullVoltage;
            step_ = -1;
            response->success = true;
            response->message = "Battery reset to full voltage.";
            RCLCPP_INFO(this->get_logger(), "Battery reset to full voltage.");
        });
        // Create a publisher that publishes my_robot_interfaces::msg::BatteryStatus messages on the "battery_status" topic
        publisher_ = this->create_publisher<my_robot_interfaces::msg::BatteryStatus>("battery_status", 1);
        


        message_.voltage = voltage_;
        message_.percentage = 100.0f;
        message_.is_charging = true; // Example charging status
        message_.status = "Charging"; // Example status  std::this_thread::sleep_for(std::chrono::seconds(3));   // #include <thread>
        // Create a timer that publishes the battery status every second
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                // in the timer
                step_++;
                if (step_ > kSteps) {
                    step_ = 0;
                }
                voltage_ = kFullVoltage - step_ * 0.1f;
                // Map voltage 10.5 V (empty) .. 12.5 V (full) to 0 .. 100 %
                // Use the integer step (not the float voltage) so 10.0% is exactly 10.0
                float pct = 100.0f * (kSteps - step_) / kSteps;
                message_.percentage = std::clamp(pct, 0.0f, 100.0f);
                message_.voltage = voltage_;
                message_.is_charging = false; // Simulate charging status change
                if (message_.percentage > 20) {
                    message_.status = "OK";
                } else if (message_.percentage >= 10.0f) {
                    message_.status = "LOW";
                } else {
                    message_.status = "CRITICAL";
                }
                RCLCPP_INFO(this->get_logger(), "%.1f V | %.1f%% | %s",
                            voltage_, message_.percentage, message_.status.c_str());
                publisher_->publish(message_);
            });


    }

private:
    rclcpp::Publisher<my_robot_interfaces::msg::BatteryStatus>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    my_robot_interfaces::msg::BatteryStatus message_;
    static constexpr float kFullVoltage = 12.5f;
    static constexpr float kEmptyVoltage = 10.5f;
    float voltage_{kFullVoltage};   // <-- HERE: the node's state
    // members
    static constexpr int kSteps = 20;   // 12.5 → 10.5 in 0.1 V steps
    int step_{0};
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}