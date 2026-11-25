#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/charge_battery.hpp"

using ChargeBattery = my_robot_interfaces::action::ChargeBattery;
using GoalHandle = rclcpp_action::ServerGoalHandle<ChargeBattery>;

class ChargerServer : public rclcpp::Node
{
public:
    ChargerServer() : Node("charger_server")
    {
        action_server_ = rclcpp_action::create_server<ChargeBattery>(
            this,
            "charge_battery",
            std::bind(&ChargerServer::goal_cb, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ChargerServer::cancel_cb, this, std::placeholders::_1),
            std::bind(&ChargerServer::accepted_cb, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(), "ChargerServer has been started.");
    }

private:
    rclcpp_action::Server<ChargeBattery>::SharedPtr action_server_;
    std::atomic<int> current_{20};
    std::atomic<bool> busy_{false};

    rclcpp_action::GoalResponse goal_cb(const rclcpp_action::GoalUUID & uuid,
                                        std::shared_ptr<const ChargeBattery::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(get_logger(), "Received request to charge battery to %d%%", goal->target_percentage);

        if (busy_) {
            RCLCPP_WARN(get_logger(), "Busy charging, rejecting new goal");
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (goal->target_percentage <= current_ || goal->target_percentage > 100) {
            RCLCPP_WARN(get_logger(), "Invalid target percentage: %d", goal->target_percentage);
            return rclcpp_action::GoalResponse::REJECT;
        }

        busy_ = true;   // Occupied
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse cancel_cb(const std::shared_ptr<GoalHandle> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(get_logger(), "Received request to cancel charging.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // Run execute() on its own thread so the spin thread stays free
    void accepted_cb(const std::shared_ptr<GoalHandle> goal_handle)
    {
        std::thread{[this, goal_handle]() { execute(goal_handle); }}.detach();
    }

    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(get_logger(), "Charging battery...");
        const int target = goal_handle->get_goal()->target_percentage;
        auto feedback = std::make_shared<ChargeBattery::Feedback>();
        auto result = std::make_shared<ChargeBattery::Result>();

        while (rclcpp::ok() && current_ < target) {
            if (goal_handle->is_canceling()) {
                result->success = false;
                result->final_percentage = current_;
                busy_ = false;   // Free
                goal_handle->canceled(result);
                RCLCPP_INFO(get_logger(), "Charging canceled at %d%%.", current_.load());
                return;
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
            current_ = std::min(current_ + 5, target);
            feedback->current_percentage = current_;
            goal_handle->publish_feedback(feedback);
        }

        result->success = true;
        result->final_percentage = current_;
        busy_ = false;   // Free
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "Charging completed.");
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChargerServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}