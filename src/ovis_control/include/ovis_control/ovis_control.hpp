// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#ifndef OVIS_CONTROL__OVIS_CONTROL_HPP_
#define OVIS_CONTROL__OVIS_CONTROL_HPP_

#include <string>
#include <vector>

#include "ovis_control/visibility_control.h"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

// Kinova includes
#include "kinova_driver/kinova_api.h"
#include "kinova_driver/kinova_comm.h"

namespace ovis_control
{

class OvisHWInterface : public hardware_interface::SystemInterface
{
public:
  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC 
  // hardware_interface::return_type prepare_command_mode_switch(
  //   const std::vector<std::string> & start_interfaces,
  //   const std::vector<std::string> & stop_interfaces) override;

  virtual ~OvisHWInterface();

private:
  enum integration_level_t : std::uint8_t
  {
    UNDEFINED = 0,
    POSITION,
    VELOCITY,
    TORQUE,
  };

  std::vector<double> hw_position_commands_;
  std::vector<double> hw_position_states_;
  std::vector<double> hw_position_init_;
  std::vector<double> hw_velocity_commands_;
  std::vector<double> hw_velocity_states_;
  std::vector<double> hw_effort_commands_;
  std::vector<double> hw_effort_states_;
  // std::vector<double> hw_accel_commands_;
  // std::vector<double> hw_accel_states_;
  kinova::KinovaComm* comm = nullptr;
  boost::recursive_mutex mApiMutex{};
  volatile bool isActive = false;


  integration_level_t control_level_;

  const rclcpp::Logger logger() const;
};

}  // namespace ovis_control

#endif  // OVIS_CONTROL__OVIS_CONTROL_HPP_
