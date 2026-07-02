// Copyright (c) 2023 Franka Robotics GmbH
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

#pragma once

#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/**
 * The joint velocity example controller.
 *
 * Extends the original sine-wave demo with an optional external command interface:
 * publish std_msgs/Float64MultiArray (7 values) to the controller's ~/commands topic
 * to override the built-in demo motion.  When no external command has been received
 * the controller falls back to the original sine-wave profile.
 */
class JointVelocityExampleController : public controller_interface::ControllerInterface {
 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::string robot_type_;
  std::string arm_prefix_;
  std::string robot_description_;
  bool is_gazebo{false};
  const int num_joints = 7;
  rclcpp::Duration elapsed_time_ = rclcpp::Duration(0, 0);

  // Smoothed velocity state (exponential low-pass filter output).
  // Seeded from actual joint velocities at on_activate() so there is no
  // step discontinuity at controller start.
  std::vector<double> current_velocities_;

  // Exponential low-pass filter coefficient α ∈ (0, 1].
  // At 1 kHz:  time constant τ ≈ dt / α = 0.001 / α
  //   α = 0.01  →  τ ≈ 100 ms  (very smooth, slow response)
  //   α = 0.05  →  τ ≈  20 ms  (good balance, default)
  //   α = 0.10  →  τ ≈  10 ms  (fast, ~1 rad/s² initial accel per 0.1 rad/s step)
  // Configurable via ROS parameter 'filter_coefficient'.
  double filter_coefficient_{0.05};

  // Optional external velocity commands (std_msgs/Float64MultiArray on ~/commands)
  // An empty buffer means "no external command received yet" → use sine-wave demo.
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_command_subscriber_;
  realtime_tools::RealtimeBuffer<std::vector<double>> velocity_commands_buffer_;
};

}  // namespace franka_example_controllers
