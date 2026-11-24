#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class CountListener : public rclcpp::Node
{
public:
    explicit CountListener(const std::string & node_name) : Node(node_name)
    {
        last_msg_time_ = now();
        timeout_sec_ = declare_parameter<double>("timeout_sec", 2.0);

        subscription_ = create_subscription<std_msgs::msg::Int32>(
            "count", 10,
            [this](std_msgs::msg::Int32::UniquePtr msg) {
                last_msg_time_ = now();
                if (publisher_silent_) {
                    RCLCPP_INFO(get_logger(), "Publisher is back");
                    publisher_silent_ = false;
                }
                RCLCPP_INFO(get_logger(), "Received: %d", msg->data);
            });

        // Registered ONCE, here in the constructor
        param_callback_handle_ = add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> & parameters) {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;

                for (const auto & param : parameters) {
                    if (param.get_name() == "timeout_sec") {
                        double value = param.as_double();
                        if (value <= 0.0) {                  // validate first
                            result.successful = false;
                            result.reason = "timeout_sec must be > 0.0";
                            return result;
                        }
                        timeout_sec_ = value;                // then apply
                        RCLCPP_INFO(get_logger(), "Updated timeout_sec to: %.1f", timeout_sec_);
                    }
                }
                return result;
            });

        // Timer does only the periodic check
        watchdog_timer_ = create_wall_timer(
            500ms,
            [this]() {
                auto elapsed = now() - last_msg_time_;
                if (elapsed.seconds() > timeout_sec_ && !publisher_silent_) {
                    RCLCPP_WARN(get_logger(), "No message on /count for %.1f seconds!", timeout_sec_);
                    publisher_silent_ = true;
                }
            });

        RCLCPP_INFO(get_logger(), "CountListener has been started.");
    }

private:
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    rclcpp::Time last_msg_time_;
    double timeout_sec_{2.0};
    bool publisher_silent_{false};
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountListener>("count_listener");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}