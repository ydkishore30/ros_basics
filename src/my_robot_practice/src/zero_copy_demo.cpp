// 1. Includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <memory>
#include <chrono>

class ZeroCopyPublisher : public rclcpp::Node
{
public:
    ZeroCopyPublisher(const rclcpp::NodeOptions & options) : Node("zero_copy_publisher",options)
    {
        RCLCPP_INFO(this->get_logger(), "ZeroCopyPublisher Node has been started.");
        auto qos = rclcpp::QoS(10);  // Use best effort QoS for big messages
        // Create a publisher to the "zero_copy_data" topic
        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("zero_copy_data", qos);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                auto msg = std::make_unique<std_msgs::msg::Float32MultiArray>();
                msg->data.resize(1000000);
                RCLCPP_INFO(get_logger(), "pub address: %p", (void*)msg.get());
                publisher_->publish(std::move(msg));
                RCLCPP_INFO(get_logger(), "Published a zero-copy message with 1,000,000 floats.");
            });
    }
    ;


private:
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

class ZeroCopySubscriber : public rclcpp::Node
{
public:
    ZeroCopySubscriber(const rclcpp::NodeOptions & options) : Node("zero_copy_subscriber",options)
    {
        RCLCPP_INFO(this->get_logger(), "ZeroCopySubscriber Node has been started.");
        auto qos = rclcpp::QoS(10);  // Use best effort QoS for big messages
        // Create a subscription to the "zero_copy_data" topic
        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "zero_copy_data",
            qos,
            [this](std_msgs::msg::Float32MultiArray::UniquePtr msg) {
                // Process the received message
                RCLCPP_INFO(get_logger(), "Received a zero-copy message with %zu floats.", msg->data.size());
                RCLCPP_INFO(get_logger(), "sub address: %p", (void*)msg.get());
            });
    }
private:
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
};  

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions().use_intra_process_comms(false);
    auto pub = std::make_shared<ZeroCopyPublisher>(options);
    auto sub = std::make_shared<ZeroCopySubscriber>(options);

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(pub);
    exec.add_node(sub);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}