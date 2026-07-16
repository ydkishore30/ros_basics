// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "hardware/my_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <string>
#include <sstream>
#include <termios.h>
#include <sys/select.h>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hardware
{
hardware_interface::CallbackReturn MyHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  RCLCPP_INFO(rclcpp::get_logger("MyHardware"), "Initializing MyHardware");

  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get serial port parameters from hardware info
  if (info_.hardware_parameters.find("serial_port") != info_.hardware_parameters.end()) {
    serial_port_name_ = info_.hardware_parameters.at("serial_port");
  } else {
    serial_port_name_ = "/dev/ttyACM0";  // Default Arduino port
  }

  if (info_.hardware_parameters.find("baud_rate") != info_.hardware_parameters.end()) {
    baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));
  } else {
    baud_rate_ = 115200;  // Default baud rate
  }

  if (info_.hardware_parameters.find("counts_per_rev") != info_.hardware_parameters.end()) {
    counts_per_rev_ = std::stod(info_.hardware_parameters.at("counts_per_rev"));
  } else {
    counts_per_rev_ = 360.0;  // Default counts per revolution
  }

  if (info_.hardware_parameters.find("wheel_radius") != info_.hardware_parameters.end()) {
    wheel_radius_ = std::stod(info_.hardware_parameters.at("wheel_radius"));
  } else {
    wheel_radius_ = 0.1;  // Default wheel radius in meters
  }

  RCLCPP_INFO(rclcpp::get_logger("MyHardware"), "Serial port: %s, Baud rate: %d, Counts per rev: %.1f, Wheel radius: %.3f",
              serial_port_name_.c_str(), baud_rate_, counts_per_rev_, wheel_radius_);

  // Get joint names from hardware info
  auto joint_names = info_.joints;
  if (joint_names.size() != 2) {
    RCLCPP_ERROR(rclcpp::get_logger("MyHardware"), "Expected 2 joints, got %zu", joint_names.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Check if joints are configured correctly
  for (const auto & joint : joint_names) {
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_ERROR(rclcpp::get_logger("MyHardware"), "Joint %s has incorrect command interfaces", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 3 ||
        joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
        joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY ||
        joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_ERROR(rclcpp::get_logger("MyHardware"), "Joint %s has incorrect state interfaces", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Initialize joint state and command vectors
  hw_positions_.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  hw_efforts_.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(joint_names.size(), std::numeric_limits<double>::quiet_NaN());

  RCLCPP_INFO(rclcpp::get_logger("MyHardware"), "MyHardware initialized successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MyHardware"), "Configuring MyHardware");

  // Initialize serial port
  try {
    serial_port_ = std::make_unique<boost::asio::serial_port>(io_service_, serial_port_name_);
    serial_port_->set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
    serial_port_->set_option(boost::asio::serial_port_base::character_size(8));
    serial_port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

    RCLCPP_INFO(rclcpp::get_logger("MyHardware"), "Serial port opened successfully");

    // Opening the port toggles DTR which resets the ESP32 (CP2102 auto-reset).
    // Wait for it to finish booting before the RT loop starts sending commands.
    RCLCPP_INFO(rclcpp::get_logger("MyHardware"), "Waiting 3s for ESP32 boot after DTR reset...");
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // Discard any garbage the ESP32 sent during boot
    int fd = serial_port_->native_handle();
    ::tcflush(fd, TCIOFLUSH);

  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("MyHardware"), "Failed to open serial port: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Reset joint state and command values
  for (size_t i = 0; i < hw_positions_.size(); ++i) {
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = 0.0;
    hw_efforts_[i] = 0.0;
    hw_commands_[i] = 0.0;
  }

  RCLCPP_INFO(rclcpp::get_logger("MyHardware"), "MyHardware configured successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MyHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MyHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn MyHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MyHardware"), "Activating MyHardware");

  // Set initial values
  for (size_t i = 0; i < hw_positions_.size(); ++i) {
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = 0.0;
    hw_efforts_[i] = 0.0;
    hw_commands_[i] = 0.0;
  }

  RCLCPP_INFO(rclcpp::get_logger("MyHardware"), "MyHardware activated successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MyHardware"), "Deactivating MyHardware");

  // Stop all movement
  for (size_t i = 0; i < hw_commands_.size(); ++i) {
    hw_commands_[i] = 0.0;
  }

  // Close serial port
  if (serial_port_ && serial_port_->is_open()) {
    serial_port_->close();
    RCLCPP_INFO(rclcpp::get_logger("MyHardware"), "Serial port closed");
  }

  RCLCPP_INFO(rclcpp::get_logger("MyHardware"), "MyHardware deactivated successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// Returns true if data is available on fd within timeout_ms, false on timeout.
static bool waitForSerial(int fd, int timeout_ms)
{
  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(fd, &rfds);
  struct timeval tv;
  tv.tv_sec  = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;
  return ::select(fd + 1, &rfds, nullptr, nullptr, &tv) > 0;
}

hardware_interface::return_type MyHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!serial_port_ || !serial_port_->is_open()) {
    RCLCPP_ERROR(rclcpp::get_logger("MyHardware"), "Serial port not available");
    return hardware_interface::return_type::ERROR;
  }

  try {
    boost::system::error_code ec;
    const double rad_per_count = 2.0 * M_PI / counts_per_rev_;
    const int fd = serial_port_->native_handle();

    // ── Encoder counts → position ────────────────────────────────────────────
    // Protocol: send "E\n", firmware replies "E <left_count> <right_count>\n"
    boost::asio::write(*serial_port_, boost::asio::buffer(std::string("E\n")));
    if (waitForSerial(fd, 500)) {
      boost::asio::streambuf buf;
      boost::asio::read_until(*serial_port_, buf, '\n', ec);
      if (!ec) {
        std::istream is(&buf);
        std::string line;
        std::getline(is, line);
        if (line.size() >= 2 && line[0] == 'E') {
          std::istringstream ss(line.substr(2));
          long left_count = 0, right_count = 0;
          if (ss >> left_count >> right_count) {
            hw_positions_[0] = static_cast<double>(left_count)  * rad_per_count;
            hw_positions_[1] = static_cast<double>(right_count) * rad_per_count;
          }
        }
      }
    } else {
      RCLCPP_WARN(rclcpp::get_logger("MyHardware"), "Encoder read timed out (E cmd)");
    }

    // ── Wheel RPM → velocity ─────────────────────────────────────────────────
    // Protocol: send "R\n", firmware replies "R <left_rpm> <right_rpm>\n"
    boost::asio::write(*serial_port_, boost::asio::buffer(std::string("R\n")));
    if (waitForSerial(fd, 500)) {
      boost::asio::streambuf buf;
      boost::asio::read_until(*serial_port_, buf, '\n', ec);
      if (!ec) {
        std::istream is(&buf);
        std::string line;
        std::getline(is, line);
        if (line.size() >= 2 && line[0] == 'R') {
          std::istringstream ss(line.substr(2));
          float left_rpm = 0.0f, right_rpm = 0.0f;
          if (ss >> left_rpm >> right_rpm) {
            // RPM → rad/s
            hw_velocities_[0] = static_cast<double>(left_rpm)  * 2.0 * M_PI / 60.0;
            hw_velocities_[1] = static_cast<double>(right_rpm) * 2.0 * M_PI / 60.0;
          }
        }
      }
    } else {
      RCLCPP_WARN(rclcpp::get_logger("MyHardware"), "RPM read timed out (R cmd)");
    }

    hw_efforts_[0] = hw_velocities_[0] * 0.1;
    hw_efforts_[1] = hw_velocities_[1] * 0.1;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("MyHardware"), "Error reading from serial port: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MyHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!serial_port_ || !serial_port_->is_open()) {
    RCLCPP_ERROR(rclcpp::get_logger("MyHardware"), "Serial port not available");
    return hardware_interface::return_type::ERROR;
  }

  try {
    // Scale rad/s → [-1, 1]: max wheel speed = 0.35 m/s / 0.035 m = 10 rad/s
    constexpr double MAX_RAD_S = 10.0;
    double left_scaled  = std::max(-1.0, std::min(1.0, hw_commands_[0] / MAX_RAD_S));
    double right_scaled = std::max(-1.0, std::min(1.0, hw_commands_[1] / MAX_RAD_S));

    // Protocol: "<left> <right>\n"  (space-separated, [-1,1])
    std::string command = std::to_string(left_scaled) + " " +
                          std::to_string(right_scaled) + "\n";

    boost::asio::write(*serial_port_, boost::asio::buffer(command));

    RCLCPP_INFO(rclcpp::get_logger("MyHardware"), "Sent to Arduino: %s", command.c_str());

  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("MyHardware"), "Error writing to serial port: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
hardware::MyHardware, hardware_interface::SystemInterface)