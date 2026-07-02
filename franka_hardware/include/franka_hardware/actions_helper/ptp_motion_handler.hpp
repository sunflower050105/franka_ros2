// Copyright (c) 2025 Franka Robotics GmbH
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

// NOTE: This file is a compatibility stub for libfranka < 0.18.0.
// AsyncPositionControlHandler is not available in libfranka 0.17.x (server protocol v9).
// PTP motion is therefore not supported with this libfranka version.

#pragma once

#include <memory>
#include <optional>
#include <string>

#include <franka_hardware/robot.hpp>
#include <franka_msgs/action/ptp_motion.hpp>

namespace franka_hardware {

/**
 * Stub PTPMotionHandler for libfranka 0.17.x (no AsyncPositionControlHandler available).
 */
class PTPMotionHandler {
 public:
  // Minimal TargetStatus enum mirroring what franka::TargetStatus provides in >=0.18
  enum class TargetStatus {
    kIdle,
    kExecuting,
    kTargetReached,
    kAborted,
  };

  struct TargetFeedback {
    TargetStatus status{TargetStatus::kAborted};
    std::optional<std::string> error_message{"PTPMotion not supported with libfranka 0.17.x"};
  };

  struct CommandResult {
    std::string motion_id;
    std::shared_ptr<franka_msgs::action::PTPMotion::Result> result;
  };

  explicit PTPMotionHandler(const std::shared_ptr<Robot>& /*robot*/) {}
  virtual ~PTPMotionHandler() = default;

  auto startNewPTPMotion(const std::shared_ptr<franka::Robot>& /*robot*/,
                         const std::shared_ptr<const franka_msgs::action::PTPMotion::Goal>& /*goal*/)
      -> CommandResult {
    auto result = std::make_shared<franka_msgs::action::PTPMotion::Result>();
    result->target_status.status = franka_msgs::msg::TargetStatus::ABORTED;
    result->error_message = "PTPMotion is not supported with libfranka 0.17.x (server protocol v9)";
    return CommandResult{"", result};
  }

  auto getFeedback(const std::string& /*motion_id*/) -> TargetFeedback {
    return TargetFeedback{};
  }

  auto cancelMotion() -> void {}
};

}  // namespace franka_hardware
