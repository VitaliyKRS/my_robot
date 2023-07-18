#include "tracked_robot_hardware/DiffDriveArduinoHardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace tracked_robot_hardware {

TRACKED_ROBOT_HARDWARE_PUBLIC hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_init(const hardware_interface::HardwareInfo &info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }


  //mConfig.left_sprocket_name = info_.hardware_parameters["left_wheel_name"];
  //mConfig.right_sprocket_name = info_.hardware_parameters["right_wheel_name"];
  //mConfig.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  //mConfig.device = info_.hardware_parameters["device"];
  //mConfig.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  //mConfig.timeout = std::stoi(info_.hardware_parameters["timeout_ms"]);
  //mConfig.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  mWheels.resize(info_.joints.size());
  


  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}
TRACKED_ROBOT_HARDWARE_PUBLIC std::vector<hardware_interface::StateInterface> DiffDriveArduinoHardware::export_state_interfaces()
{
   RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "export_state_interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Adding position state interface: %s", info_.joints[i].name.c_str());
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &mWheels[i].getPosition()
            )
        );
    }
    for (size_t i = 0; i < info_.joints.size(); i++) {
        RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Adding velocity state interface: %s", info_.joints[i].name.c_str());
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &mWheels[i].getVelocity()
            )
        );
    }

  return state_interfaces;
}
TRACKED_ROBOT_HARDWARE_PUBLIC std::vector<hardware_interface::CommandInterface> DiffDriveArduinoHardware::export_command_interfaces()
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "export_command_interfaces");

    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Adding velocity command interface: %s", info_.joints[i].name.c_str());
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &mWheels[i].getCommand()
            )
        );
    }
    return command_interfaces;
}
TRACKED_ROBOT_HARDWARE_PUBLIC hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_configure(const rclcpp_lifecycle::State &previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Configuring ...please wait...");

  // Fake connection

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}
TRACKED_ROBOT_HARDWARE_PUBLIC hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_cleanup(const rclcpp_lifecycle::State &previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Cleaning up ...please wait...");
   
   // Fake cleanup

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

TRACKED_ROBOT_HARDWARE_PUBLIC hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_activate(const rclcpp_lifecycle::State &previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Activating ...please wait...");

  // Set some fake default values


  for (size_t i = 0; i < info_.joints.size(); i++) {
        mWheels[i].setCommand(0);
        mWheels[i].setPosition(0);
        mWheels[i].setVelocity(0);
    }

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}
TRACKED_ROBOT_HARDWARE_PUBLIC hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}
TRACKED_ROBOT_HARDWARE_PUBLIC hardware_interface::return_type DiffDriveArduinoHardware::read(const rclcpp::Time &time, const rclcpp::Duration &period)
{
   for (size_t i = 0; i < info_.joints.size(); i++) {
         RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "read pos %f - vel %f", mWheels[i].getPosition(), mWheels[i].getVelocity());
    }

  
  auto new_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = new_time - time_;
  double deltaSeconds = diff.count();
  time_ = new_time;

    // Force the wheel position
    for (size_t i = 0; i < info_.joints.size(); i++) {
        mWheels[i].setPosition(mWheels[i].getPosition() + mWheels[i].getVelocity() + deltaSeconds);
    }

  return hardware_interface::return_type::OK;
}
TRACKED_ROBOT_HARDWARE_PUBLIC hardware_interface::return_type DiffDriveArduinoHardware::write(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    // Set the wheel velocities to directly match what is commanded
   for (size_t i = 0; i < info_.joints.size(); i++) {
        RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "write %f", mWheels[i].getCommand());
    }
 

 for (size_t i = 0; i < info_.joints.size(); i++) {
        mWheels[i].setVelocity(double(mWheels[i].getCommand()));
    }
  return hardware_interface::return_type::OK;
}
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  tracked_robot_hardware::DiffDriveArduinoHardware, hardware_interface::SystemInterface)