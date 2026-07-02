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

#include <franka_example_controllers/default_robot_behavior_utils.hpp>
#include <franka_example_controllers/fr3/joint_velocity_example_controller.hpp>
#include <franka_example_controllers/robot_utils.hpp>

#include <algorithm>
#include <cassert>
#include <exception>
#include <rclcpp/logging.hpp>
#include <string>
#include <vector>

#include <Eigen/Eigen>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
JointVelocityExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_prefix_ + robot_type_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointVelocityExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_prefix_ + robot_type_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_prefix_ + robot_type_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::return_type JointVelocityExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {

  const std::vector<double>* external_cmds = velocity_commands_buffer_.readFromRT();

  // Determine target velocities: external command or zero (safe stop)
  std::vector<double> target_velocities(num_joints, 0.0);
  if (external_cmds != nullptr && static_cast<int>(external_cmds->size()) == num_joints) {
    target_velocities = *external_cmds;
  } else {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                         "No valid external velocity commands received. Stopping robot.");
  }

  // === Exponential low-pass filter ===
  //
  // v_out[k] = v_out[k-1] + α * (v_target - v_out[k-1])
  //
  // At 1 kHz with α = 0.02 (default):
  //   Peak effective accel ≈ α * |Δv| / dt
  //   For |Δv| = 0.1 rad/s  →  0.02 * 0.1 / 0.001 = 2 rad/s²  (well within FR3 ~10 rad/s² limit)
  //   For |Δv| = 0.3 rad/s  →  0.02 * 0.3 / 0.001 = 6 rad/s²  (still safe)
  //
  // Because the filter output is an exponential rise, the acceleration itself
  // decays smoothly from its peak — there is no abrupt step in acceleration —
  // which avoids the joint_motion_generator_acceleration_discontinuity reflex.
  const double alpha = filter_coefficient_;
  for (int i = 0; i < num_joints; ++i) {
    current_velocities_[i] += alpha * (target_velocities[i] - current_velocities_[i]);
  }

  // Send the smoothed velocities to the hardware
  for (int i = 0; i < num_joints; ++i) {
    if (!command_interfaces_[i].set_value(current_velocities_[i])) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to set command interface %s",
                   command_interfaces_[i].get_name().c_str());
      return controller_interface::return_type::ERROR;
    }
  }

  // Optional low-frequency logging (every ~0.5 s at 1 kHz)
  static int counter = 0;
  if (++counter % 500 == 0) {
    RCLCPP_DEBUG(get_node()->get_logger(),
                 "Filtered velocities: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                 current_velocities_[0], current_velocities_[1], current_velocities_[2],
                 current_velocities_[3], current_velocities_[4], current_velocities_[5],
                 current_velocities_[6]);
  }

  return controller_interface::return_type::OK;
}

CallbackReturn JointVelocityExampleController::on_init() {
  try {
    auto_declare<std::string>("arm_prefix", "");
    auto_declare<bool>("gazebo", false);
    // Low-pass filter coefficient α ∈ (0,1].
    // Smaller α → smoother / slower; larger α → faster / less smooth.
    // Default 0.02 keeps peak accel ≤ ~6 rad/s² even for 0.3 rad/s step commands.
    auto_declare<double>("filter_coefficient", 0.02);
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception during init: %s\n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointVelocityExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {

  is_gazebo = get_node()->get_parameter("gazebo").as_bool();

  // Get robot description and type
  auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(
      get_node(), "robot_state_publisher");
  parameters_client->wait_for_service();
  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();

  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
    if (robot_description_.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "robot_description parameter is empty.");
      return CallbackReturn::ERROR;
    }
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description.");
    return CallbackReturn::ERROR;
  }

  robot_type_ = robot_utils::getRobotNameFromDescription(
      robot_description_, get_node()->get_logger());
  arm_prefix_ = get_node()->get_parameter("arm_prefix").as_string();
  arm_prefix_ = arm_prefix_.empty() ? "" : arm_prefix_ + "_";

  // Read filter coefficient; clamp to (0, 1]
  filter_coefficient_ = std::max(1e-4, std::min(1.0,
      get_node()->get_parameter("filter_coefficient").as_double()));
  RCLCPP_INFO(get_node()->get_logger(),
              "Velocity low-pass filter: α = %.4f  (time-constant ≈ %.1f ms at 1 kHz)",
              filter_coefficient_, 1.0 / filter_coefficient_);

  // Set default collision behavior on real robot
  if (!is_gazebo) {
    auto client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
        "service_server/set_full_collision_behavior");
    auto request = DefaultRobotBehavior::getDefaultCollisionBehaviorRequest();
    auto future_result = client->async_send_request(request);
    future_result.wait_for(robot_utils::time_out);

    if (!future_result.get()) {
      RCLCPP_FATAL(get_node()->get_logger(), "Failed to set default collision behavior.");
      return CallbackReturn::ERROR;
    }
  }

  // Subscribe to external velocity commands
  velocity_command_subscriber_ =
      get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
          "~/commands", rclcpp::QoS(10).best_effort(),
          [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            if (static_cast<int>(msg->data.size()) == num_joints) {
              velocity_commands_buffer_.writeFromNonRT(
                  std::vector<double>(msg->data.begin(), msg->data.end()));
            } else {
              RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                  "Received %zu values, expected %d. Ignoring.", msg->data.size(), num_joints);
            }
          });

  RCLCPP_INFO(get_node()->get_logger(),
              "JointVelocityExampleController configured. Subscribed to '~/commands'. "
              "Robot will stop safely if no commands are received.");

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointVelocityExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {

  // Seed the filter state and the command buffer with the actual measured joint
  // velocities so the very first update() cycle sees zero error → no spike.
  current_velocities_.resize(num_joints, 0.0);
  for (int i = 0; i < num_joints; ++i) {
    current_velocities_[i] = state_interfaces_[2 * i + 1].get_value();
  }

  velocity_commands_buffer_.writeFromNonRT(current_velocities_);

  RCLCPP_INFO(get_node()->get_logger(),
              "Controller activated. Seeded filter with: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
              current_velocities_[0], current_velocities_[1], current_velocities_[2],
              current_velocities_[3], current_velocities_[4], current_velocities_[5],
              current_velocities_[6]);

  return CallbackReturn::SUCCESS;
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointVelocityExampleController,
                       controller_interface::ControllerInterface)
