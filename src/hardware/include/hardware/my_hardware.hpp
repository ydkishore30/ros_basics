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

#ifndef HARDWARE_MY_HARDWARE_HPP_
#define HARDWARE_MY_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace hardware
{
class MyHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MyHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters for the DiffBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;

  // Serial communication
  boost::asio::io_service io_service_;
  std::unique_ptr<boost::asio::serial_port> serial_port_;
  std::string serial_port_name_;
  uint32_t baud_rate_;

  // Encoder parameters
  double counts_per_rev_;
  double wheel_radius_;

  // Joint state and command vectors
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  std::vector<double> hw_commands_;

  // Velocity computed from encoder deltas (firmware has no R command)
  std::vector<double> prev_positions_;

  // Per-wheel velocity PID (feed-forward + PI + D on measurement), output in [-1, 1]
  bool use_pid_{true};
  double pid_offset_{0.0}; // output needed to overcome motor dead zone / static friction
  double pid_kff_{0.1};    // output per rad/s of target above the dead zone
  double pid_kp_{0.05};    // output per rad/s of error
  double pid_ki_{0.2};     // output per (rad/s * s) of accumulated error
  double pid_kd_{0.0};     // output per rad/s^2 of filtered velocity change
  double pid_i_band_{0.5}; // integrate only when |error| < band * |target| (integral separation)
  double vel_filter_alpha_{0.3};  // EMA weight of the newest velocity sample
  bool encoder_ok_{false};        // last read() got a fresh encoder reply
  std::vector<double> vel_filtered_;
  std::vector<double> prev_vel_filtered_;
  std::vector<double> pid_integral_;
  std::vector<double> prev_targets_;
  rclcpp::Time prev_read_time_{0, 0, RCL_STEADY_TIME};

  // Persistent serial receive buffer: read_until may pull in several lines at
  // once, and bytes past the first '\n' must survive to the next read.
  boost::asio::streambuf serial_buf_;

  // IMU publisher (BNO data comes from same serial port)
  rclcpp::Node::SharedPtr imu_node_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  std::string imu_frame_id_{"imu_link"};
  uint32_t imu_poll_counter_{0};
  double imu_yaw_{0.0};
  bool have_imu_yaw_{false};

  void publishImuLine(const std::string & line);
  void updateOrientation(const std::string & line);
};

}  // namespace hardware

#endif  // HARDWARE_MY_HARDWARE_HPP_