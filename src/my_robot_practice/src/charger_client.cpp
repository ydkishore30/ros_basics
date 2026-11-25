#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/action/charge_battery.hpp"

#include <chrono>
#include "rclcpp_action/rclcpp_action.hpp"


using ChargeBattery = my_robot_interfaces::action::ChargeBattery;
using GoalHandle = rclcpp_action::ClientGoalHandle<ChargeBattery>;

class ChargerClient : public rclcpp::Node
{
public:
    ChargerClient() : Node("charger_client")
    {
        client_ = rclcpp_action::create_client<ChargeBattery>(this, "charge_battery");
        RCLCPP_INFO(get_logger(), "ChargerClient has been started.");
        timer_ = create_wall_timer(
            std::chrono::seconds(5),
            [this]() {
                if (!client_->wait_for_action_server(std::chrono::seconds(1))) {
                    RCLCPP_WARN(get_logger(), "Waiting for action server...");
                    return;
                }
                timer_->cancel();  // cancel the timer, we don't need it anymore
                send_goal();
            });

    }


    private:
    rclcpp_action::Client<ChargeBattery>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    void send_goal()
    {
    // 1. Goal message
    auto goal_msg = ChargeBattery::Goal();
    goal_msg.target_percentage = 80;

    // 2. Options with 3 callbacks
    auto options = rclcpp_action::Client<ChargeBattery>::SendGoalOptions();

    options.goal_response_callback =
        [this](const GoalHandle::SharedPtr & goal_handle) {
            // goal_handle == nullptr అయితే rejected, లేకపోతే accepted
            if (!goal_handle) {
                RCLCPP_ERROR(get_logger(), "Goal rejected by server");
                rclcpp::shutdown();   // no result will come, so exit now
            } else {
                RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
            }
        };

    options.feedback_callback =
        [this](GoalHandle::SharedPtr, const std::shared_ptr<const ChargeBattery::Feedback> feedback) {
            // feedback->current_percentage print చేయండి
            RCLCPP_INFO(get_logger(), "Current battery percentage: %d%%", feedback->current_percentage);
        };

    options.result_callback =
        [this](const GoalHandle::WrappedResult & result) {
            // result.code: rclcpp_action::ResultCode::SUCCEEDED / CANCELED / ABORTED
            // result.result->final_percentage print చేయండి
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(get_logger(), "Charging done: %d%%", result.result->final_percentage);
                } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
                    RCLCPP_WARN(get_logger(), "Canceled at %d%%", result.result->final_percentage);
                } else {
                    RCLCPP_ERROR(get_logger(), "Charging failed");
                }
                RCLCPP_INFO(get_logger(), "Final battery percentage: %d%%", result.result->final_percentage);

                rclcpp::shutdown();   // done, so exit the program
        };

    // 3. Send
    client_->async_send_goal(goal_msg, options);
}
    
};
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChargerClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}