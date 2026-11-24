#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "my_robot_interfaces/msg/battery_status.hpp"
#include "my_robot_interfaces/srv/set_charge_level.hpp"

using namespace std::chrono_literals;

class BatteryNode : public rclcpp::Node
{
public:
    BatteryNode() : Node("battery_node")
    {
        publisher_ = create_publisher<my_robot_interfaces::msg::BatteryStatus>("battery_status", 10);

        reset_srv_ = create_service<std_srvs::srv::Trigger>(
            "reset_battery",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                (void)request;
                step_ = 0;   // 0 = full
                response->success = true;
                response->message = "Battery reset to full.";
                RCLCPP_INFO(get_logger(), "Battery reset to full.");
            });

        charge_srv_ = create_service<my_robot_interfaces::srv::SetChargeLevel>(
            "set_charge_level",
            [this](const std::shared_ptr<my_robot_interfaces::srv::SetChargeLevel::Request> request,
                   std::shared_ptr<my_robot_interfaces::srv::SetChargeLevel::Response> response) {
                if (request->percentage < 0 || request->percentage > 100) {
                    response->success = false;
                    response->message = "percentage must be 0-100, got " +
                                        std::to_string(request->percentage);
                    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
                    return;
                }
                // 100% -> step 0, 0% -> step 20 (each step is 5%)
                step_ = (100 - request->percentage) * kSteps / 100;
                response->success = true;
                response->message = "Charge set to " + std::to_string(request->percentage) + "%";
                RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
            });

        timer_ = create_wall_timer(
            1s,
            [this]() {
                // 1. Build and publish the CURRENT state
                my_robot_interfaces::msg::BatteryStatus msg;
                msg.voltage = kFullVoltage - step_ * 0.1f;
                msg.percentage = 100.0f * (kSteps - step_) / kSteps;
                msg.is_charging = false;

                if (msg.percentage > 20.0f) {
                    msg.status = "OK";
                } else if (msg.percentage > 10.0f) {
                    msg.status = "LOW";
                } else {
                    msg.status = "CRITICAL";
                }

                RCLCPP_INFO(get_logger(), "%.1f V | %.1f%% | %s",
                            msg.voltage, msg.percentage, msg.status.c_str());
                publisher_->publish(msg);

                // 2. THEN advance for the next tick
                step_++;
                if (step_ > kSteps) {
                    step_ = 0;
                }
            });

        RCLCPP_INFO(get_logger(), "BatteryNode has been started.");
    }

private:
    static constexpr float kFullVoltage = 12.5f;
    static constexpr int kSteps = 20;   // 12.5 V → 10.5 V in 0.1 V steps

    rclcpp::Publisher<my_robot_interfaces::msg::BatteryStatus>::SharedPtr publisher_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
    rclcpp::Service<my_robot_interfaces::srv::SetChargeLevel>::SharedPtr charge_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
    int step_{0};   // the only state: 0 = full, 20 = empty
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}