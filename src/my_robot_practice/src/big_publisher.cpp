#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <chrono>

class BigPublisher : public rclcpp::Node
{
public:
    BigPublisher() : Node("big_publisher")
    {
        RCLCPP_INFO(this->get_logger(), "BigPublisher Node has been started.");
        auto qos = rclcpp::QoS(10);  // Use best effort QoS for big messages

        // Create a publisher to the "big_data" topic
        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("big_data", qos);
        timer_created_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
            
                auto msg = std::make_unique<std_msgs::msg::Float32MultiArray>();
                msg->data.resize(1000000);
                publisher_->publish(std::move(msg));

                RCLCPP_INFO(get_logger(), "msg is %s", msg ? "still valid" : "nullptr");    
                    });

    }   
    private:
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_created_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BigPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
