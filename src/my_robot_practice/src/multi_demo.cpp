#include "rclcpp/rclcpp.hpp"


class MultiDemoNode : public rclcpp::Node
{
public:
    MultiDemoNode() : Node("multi_demo_node")
    {
        RCLCPP_INFO(this->get_logger(), "MultiDemoNode has been started.");
        group_a_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        group_b_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        
        timer_a = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                auto tid = std::hash<std::thread::id>{}(std::this_thread::get_id());
                RCLCPP_INFO(get_logger(), "A start (thread %zu)", tid);
                // std::this_thread::sleep_for(std::chrono::seconds(2));
                RCLCPP_INFO(get_logger(), "A end (thread %zu)", tid);
                // timer_a->cancel();  // Start with the timer stopped
                
                for (int i = 0; i < 1000000; i++) {
                    counter_++;
                }
                RCLCPP_INFO(get_logger(), "A done, counter = %d", counter_);
            }, group_a_);

        timer_b = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                auto tid = std::hash<std::thread::id>{}(std::this_thread::get_id());
                RCLCPP_INFO(get_logger(), "B start (thread %zu)", tid);
                // std::this_thread::sleep_for(std::chrono::seconds(1));
                RCLCPP_INFO(get_logger(), "B end (thread %zu)", tid);
                // timer_b->cancel();  // Start with the timer stopped
                for (int i = 0; i < 1000000; i++) {
                    counter_++;
                }
                RCLCPP_INFO(get_logger(), "B done, counter = %d", counter_);
            }, group_b_);

    }
    private:
    rclcpp::TimerBase::SharedPtr timer_a;
    rclcpp::TimerBase::SharedPtr timer_b;
    rclcpp::CallbackGroup::SharedPtr group_a_;
    rclcpp::CallbackGroup::SharedPtr group_b_;
    int counter_{0};
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MultiDemoNode>();
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);
    executor->spin();
    rclcpp::shutdown();
    return 0;
}
        

