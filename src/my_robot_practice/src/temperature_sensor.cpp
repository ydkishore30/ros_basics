#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/temperature.hpp"
#include <chrono>
#include <memory>

class temperature_sensor : public rclcpp::Node
{
public:
    temperature_sensor() : Node("temperature_sensor")
    {
        RCLCPP_INFO(this->get_logger(), "Temperature Sensor Node has been started.");

        // Create a publisher to the "temperature" topic
        publisher_ = this->create_publisher<my_robot_interfaces::msg::Temperature>("temperature", 1);
        warm_threshold_ = declare_parameter<double>("warm_threshold", 35.0);
        hot_threshold_ = declare_parameter<double>("hot_threshold", 38.0);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                auto message = my_robot_interfaces::msg::Temperature();

                // RCLCPP_INFO(this->get_logger(), "Publishing: %.2f C | %s", message.celsius, message.status.c_str());

                if (temperature_ > 40.0)
                {
                    temperature_ = 25.0;
                }
                

                if (temperature_ < warm_threshold_)
                {
                    message.status = "NORMAL";
                }
                else if (temperature_ <= hot_threshold_ && temperature_ >= warm_threshold_)
                {
                    message.status = "WARM";
                }else
                {
                    message.status = "HOT";
                }
                message.celsius = temperature_;
                // message.temperature = temperature_;
                
                RCLCPP_INFO(this->get_logger(), "Publishing: %.2f C | %s", message.celsius, message.status.c_str());

                publisher_->publish(message);
                temperature_ += 0.5;

            });
    }

private:
    rclcpp::Publisher<my_robot_interfaces::msg::Temperature>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double temperature_{25.0};  // Initial temperature
    double warm_threshold_{35.0};
    double hot_threshold_{38.0};    
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<temperature_sensor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
