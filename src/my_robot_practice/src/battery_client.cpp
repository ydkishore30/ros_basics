#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_charge_level.hpp"

using namespace std::chrono_literals;
using SetChargeLevel = my_robot_interfaces::srv::SetChargeLevel;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    if (argc < 2) {
        std::cerr << "Usage: battery_client <percentage>\n";
        rclcpp::shutdown();
        return 1;
    }

    // 1. A node, so the client has a connection to ROS
    auto node = rclcpp::Node::make_shared("battery_client");

    // 2. The client, created from the node
    auto client = node->create_client<SetChargeLevel>("set_charge_level");

    // 3. Wait until the server exists
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {   // Ctrl+C while waiting
            rclcpp::shutdown();
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "Waiting for service /set_charge_level...");
    }

    // 4. Build the request
    auto request = std::make_shared<SetChargeLevel::Request>();
    request->percentage = std::stoi(argv[1]);

    // 5. Send it: returns immediately with a future
    auto future = client->async_send_request(request);

    // 6. Spin until the response arrives
    if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = future.get();
        RCLCPP_INFO(node->get_logger(), "success: %s | %s",
                    response->success ? "true" : "false",
                    response->message.c_str());
    } else {
        RCLCPP_ERROR(node->get_logger(), "Service call failed");
    }

    rclcpp::shutdown();
    return 0;
}