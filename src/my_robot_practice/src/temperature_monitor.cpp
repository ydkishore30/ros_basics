#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/temperature.hpp"
#include <chrono>
#include <memory>

class temperature_sensor : public rclcpp::Node
{
public:
    temperature_sensor() : Node("temperature_monitor")
    {
        RCLCPP_INFO(this->get_logger(), "Temperature Monitor Node has been started.");

        // Create a subscription to the "temperature" topic
        subscription_ = this->create_subscription<my_robot_interfaces::msg::Temperature>(
            "temperature", 1, [this](const my_robot_interfaces::msg::Temperature::SharedPtr msg) {

                if (msg->status == last_status_) {
                    return;   // మారలేదు, log చేయకు
                }
                last_status_ = msg->status;

                if (msg->celsius > 38.0)
                {
                    RCLCPP_ERROR(this->get_logger(), "Temperature is HOT: %.2f C | %s", msg->celsius, msg->status.c_str());
                }
                else if (msg->celsius >= 35.0 && msg->celsius <= 38.0)
                {
                    RCLCPP_WARN(this->get_logger(), "Temperature is WARM: %.2f C | %s", msg->celsius, msg->status.c_str());
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Temperature is NORMAL: %.2f C | %s", msg->celsius, msg->status.c_str());
                }
                // RCLCPP_INFO(this->get_logger(), "Received temperature: %.2f C | %s", msg->celsius, msg->status.c_str());
            });


    }

private:
    rclcpp::Subscription<my_robot_interfaces::msg::Temperature>::SharedPtr subscription_;
    std::string last_status_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<temperature_sensor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
