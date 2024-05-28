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

#include <limits>
#include <vector>

#include "ovis_control/ovis_control.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono;

namespace ovis_control
{
    hardware_interface::CallbackReturn OvisHWInterface::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        {
            RCLCPP_ERROR_ONCE(rclcpp::get_logger("ovis_control"), "Init failed");
            return CallbackReturn::ERROR;
        }
        hw_position_commands_.resize(info_.joints.size(), 0);
        hw_position_states_.resize(info_.joints.size(), 0);
        hw_position_init_.resize(info_.joints.size(), 0);
        hw_velocity_commands_.resize(info_.joints.size(), 0);
        hw_velocity_states_.resize(info_.joints.size(), 0);
        hw_effort_commands_.resize(info_.joints.size(), 0);
        hw_effort_states_.resize(info_.joints.size(), 0);
        // hw_accel_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        // hw_accel_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        // try
        // {
        //     comm = new kinova::KinovaComm(mApiMutex, info);
        // }
        // catch(const std::exception& e)
        // {
        //     return CallbackReturn::ERROR;
        // }

        RCLCPP_INFO_ONCE(rclcpp::get_logger("ovis_control"), "Init success");

        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn OvisHWInterface::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> OvisHWInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_states_[i]));
        }

        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_states_[i]));
        }

        // for (size_t i = 0; i < info_.joints.size(); ++i)
        // {
        //     state_interfaces.emplace_back(hardware_interface::StateInterface(
        //         info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_effort_states_[i]));
        // }

        // for (size_t i = 0; i < info_.joints.size(); ++i)
        // {
        //     state_interfaces.emplace_back(hardware_interface::StateInterface(
        //         info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_accel_states_[i]));
        // }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> OvisHWInterface::export_command_interfaces()
    {

        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_states_[i]));
        }

        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_states_[i]));
        }

        // for (size_t i = 0; i < info_.joints.size(); ++i)
        // {
        //     command_interfaces.emplace_back(hardware_interface::CommandInterface(
        //         info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_effort_states_[i]));
        // }

        // for (size_t i = 0; i < info_.joints.size(); ++i)
        // {
        //     command_interfaces.emplace_back(hardware_interface::CommandInterface(
        //         info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_accel_states_[i]));
        // }

        return command_interfaces;
    }

    hardware_interface::CallbackReturn OvisHWInterface::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        try
        {
            RCLCPP_INFO(logger(), "Activating");
            comm = new kinova::KinovaComm(mApiMutex, this->info_);
            RCLCPP_INFO(logger(), "Activated");
        }
        catch (const kinova::KinovaCommException &e)
        {
            RCLCPP_ERROR(logger(), e.what());
            return CallbackReturn::ERROR;
        }

        isActive = true;

        kinova::KinovaAngles angles;
        comm->getJointAngles(angles);
        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            hw_position_init_.at(i) = (double)angles[i];
        }
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn OvisHWInterface::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        isActive = false;
        if (comm != nullptr)
            delete comm;
        comm = nullptr;

        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type OvisHWInterface::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        static rclcpp::Clock clk = rclcpp::Clock();
        if (!isActive)
            return hardware_interface::return_type::ERROR;
        kinova::KinovaAngles angles;
        try
        {
            RCLCPP_INFO_THROTTLE(rclcpp::get_logger(get_name()), clk, 1000, "Getting angles");
            comm->getJointAngles(angles);
            RCLCPP_INFO_STREAM_THROTTLE(rclcpp::get_logger(get_name()), clk, 1000, "Angles at" << "\n1:" << angles.Actuator1 << "\n2:" << angles.Actuator2 << "\n3:" << angles.Actuator3 << "\n4:" << angles.Actuator4 << "\n5:" << angles.Actuator5 << "\n6:" << angles.Actuator6);

            for (size_t i = 0; i < info_.joints.size(); ++i)
            {
                hw_position_states_.at(i) = (double)angles[i] - hw_position_init_.at(i);
            }

            comm->getJointVelocities(angles);
            for (size_t i = 0; i < info_.joints.size(); ++i)
            {
                hw_velocity_states_.at(i) = (double)angles[i];
            }

            // comm->getJointTorques(angles);
            // for (size_t i = 0; i < info_.joints.size(); ++i)
            // {
            //     hw_effort_states_.at(i) = (double)angles[i];
            // }
        }
        catch (const kinova::KinovaCommException &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger(get_name()), e.what());
            return hardware_interface::return_type::ERROR;
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type OvisHWInterface::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        if (!isActive)
            return hardware_interface::return_type::ERROR;
        kinova::KinovaAngles angles;
        try
        {
            for (int i = 0; i < info_.joints.size(); ++i)
            {
                switch (control_level_)
                {
                case POSITION:
                        angles[i] = (float)hw_position_commands_.at(i) + hw_position_init_.at(i);
                    break;
                case VELOCITY:
                        angles[i] = (float)hw_velocity_commands_.at(i);
                    break;
                default:
                    return hardware_interface::return_type::ERROR;
                }
            }

            // RCLCPP_INFO_STREAM(rclcpp::get_logger(get_name()), "Setting angles to" <<
            //     "\n1:" << angles.Actuator1 <<
            //     "\n2:" << angles.Actuator2 <<
            //     "\n3:" << angles.Actuator3 <<
            //     "\n4:" << angles.Actuator4 <<
            //     "\n5:" << angles.Actuator5 <<
            //     "\n6:" << angles.Actuator6);
            switch (control_level_)
            {
            case POSITION:
                    comm->setJointAngles(angles);
                break;
            case VELOCITY:
                    comm->setJointVelocities(angles);
                break;
            default:
                return hardware_interface::return_type::ERROR;
            }
            // RCLCPP_INFO(logger(), "Angles set!");
        }
        catch (const kinova::KinovaCommException &e)
        {
            RCLCPP_ERROR(logger(), e.what());
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }

    // hardware_interface::return_type OvisHWInterface::prepare_command_mode_switch(
    //     const std::vector<std::string> &start_interfaces,
    //     const std::vector<std::string> &stop_interfaces)
    // {
    //     // Prepare for new command modes
    //     // std::vector<integration_level_t> new_modes = {};
    //     // for (std::string key : start_interfaces)
    //     // {
    //     //     for (std::size_t i = 0; i < info_.joints.size(); i++)
    //     //     {
    //     //         if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION)
    //     //         {
    //     //             new_modes.push_back(integration_level_t::POSITION);
    //     //         }
    //     //         if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY)
    //     //         {
    //     //             new_modes.push_back(integration_level_t::VELOCITY);
    //     //         }
    //     //         // if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_ACCELERATION)
    //     //         // {
    //     //         //     new_modes.push_back(integration_level_t::ACCELERATION);
    //     //         // }
    //     //     }
    //     // }
    //     // Stop motion on all relevant joints that are stopping
    //     // for (std::string key : stop_interfaces)
    //     // {
    //     //     for (std::size_t i = 0; i < info_.joints.size(); i++)
    //     //     {
    //     //         if (key.find(info_.joints[i].name) != std::string::npos)
    //     //         {
    //     //             hw_velocity_commands_[i] = 0;
    //     //             // control_level_ = integration_level_t::UNDEFINED; // Revert to undefined
    //     //         }
    //     //     }
    //     // }
    //     std::string key = start_interfaces[0];
    //     if (key == info_.joints[0].name + "/" + hardware_interface::HW_IF_POSITION)
    //     {
    //         control_level_ = integration_level_t::POSITION;
    //     }
    //     if (key == info_.joints[0].name + "/" + hardware_interface::HW_IF_VELOCITY)
    //     {
    //         control_level_ =  integration_level_t::VELOCITY;
    //     }
    //     return hardware_interface::return_type::OK;
    // }

    OvisHWInterface::~OvisHWInterface()
    {
        if (comm != nullptr)
            delete comm;
        comm = nullptr;
    }

    const rclcpp::Logger OvisHWInterface::logger() const
    {
        return rclcpp::get_logger(get_name());
    }
} // namespace ovis_control

#include "pluginlib/class_list_macros.hpp"
#include "ovis_control/ovis_control.hpp"

PLUGINLIB_EXPORT_CLASS(
    ovis_control::OvisHWInterface, hardware_interface::SystemInterface)
