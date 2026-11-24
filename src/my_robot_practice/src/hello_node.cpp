    #include "rclcpp/rclcpp.hpp"
    #include <memory>
    #include "std_msgs/msg/int32.hpp"
    #include <chrono>
    #include <string>

    using namespace std::chrono_literals;

    class HelloNode : public rclcpp::Node
    {
    public:
        HelloNode() : Node("hello_node")
        {
            RCLCPP_INFO(this->get_logger(), "HelloNode has been started.");

            // Create a publisher that publishes std_msgs::msg::Int32 messages on the "count" topic
            publisher_ = this->create_publisher<std_msgs::msg::Int32>("count", 10);
            declare_parameter("period_ms", 1000);
            period_ms_ = get_parameter("period_ms").as_int();

            create_timer();
            param_callback_handle_ = this->add_on_set_parameters_callback(
                [this](const std::vector<rclcpp::Parameter> & parameters) {
                    for (const auto & param : parameters) {
                        if (param.get_name() == "period_ms") {
                            period_ms_ = param.as_int();
                            RCLCPP_INFO(this->get_logger(), "Updated period_ms to: %d", period_ms_);
                            create_timer(); // Recreate the timer with the new period
                        }
                    }
                    rcl_interfaces::msg::SetParametersResult result;
                    result.successful = true;
                    return result;
                }
            );
        }
        private:
        void create_timer()
        {
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(period_ms_),
                [this]() {
                    count_++;

                    auto message = std_msgs::msg::Int32();
                    message.data = count_;
                    publisher_->publish(message);

                    RCLCPP_INFO(this->get_logger(), "Publishing: %d", message.data);
                }
            );
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
        int count_{0};
        int period_ms_{1000};
        OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    };

    int main(int argc, char * argv[])
    {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<HelloNode>();
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    }
