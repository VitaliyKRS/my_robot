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

#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/visibility_control.h>
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "tracked_robot_hardware/HardwareCompiler.h"
#include "tracked_robot_hardware/Config.h"
#include "tracked_robot_hardware/Wheel.h"
#include "tracked_robot_hardware/SerialPortWrapper.hpp"

namespace tracked_robot_hardware {

class DiffDriveArduinoHardware : public hardware_interface::SystemInterface
{

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffDriveArduinoHardware)

  TRACKED_ROBOT_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  TRACKED_ROBOT_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  TRACKED_ROBOT_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  TRACKED_ROBOT_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  TRACKED_ROBOT_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;


  TRACKED_ROBOT_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  TRACKED_ROBOT_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  TRACKED_ROBOT_HARDWARE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  TRACKED_ROBOT_HARDWARE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private: 
  std::vector<Wheel> mWheels;
  Config mConfig;
  SerialPortWrapper mSerial;
  std::chrono::time_point<std::chrono::system_clock> time_;
};

}