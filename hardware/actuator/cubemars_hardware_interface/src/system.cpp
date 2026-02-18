#include "cubemars_hardware/system.hpp"

#include <cmath>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cubemars_hardware
{
CubeMarsSystemHardware::~CubeMarsSystemHardware()
{
  // If the controller manager is shutdown via Ctrl + C
  on_cleanup(rclcpp_lifecycle::State());
}

hardware_interface::CallbackReturn CubeMarsSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters.count("can_interface") != 0) {
    can_itf_ = info_.hardware_parameters.at("can_interface");
  } else {
    RCLCPP_FATAL(
      rclcpp::get_logger("CubeMarsSystemHardware"),
      "No can_interface specified in URDF");
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_temperatures_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_accelerations_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  control_mode_.resize(info_.joints.size(), control_mode_t::UNDEFINED);
  internal_gear_ratios_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  external_gear_ratios_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  
  // Initialize gripper-specific variables
  grasp_detect_.resize(info_.joints.size(), false);
  grasp_detected_.resize(info_.joints.size(), false);
  target_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    const auto & joint = info_.joints[i];
    if (joint.parameters.count("can_id") != 0 &&
      joint.parameters.count("kt") != 0 &&
      joint.parameters.count("pole_pairs") != 0 &&
      joint.parameters.count("gear_ratio") != 0)
    {
      can_ids_.emplace_back(std::stoul(joint.parameters.at("can_id")));
      torque_constants_.emplace_back(std::stod(joint.parameters.at("kt")));
      internal_gear_ratios_[i] = std::stod(joint.parameters.at("gear_ratio"));
      double erpm_conversion = std::stoi(joint.parameters.at("pole_pairs")) * 
        internal_gear_ratios_[i] * 60 / (2 * M_PI);
      erpm_conversions_.emplace_back(erpm_conversion);

      if (joint.parameters.count("external_gear_ratio") != 0)
      {
        external_gear_ratios_[i] = std::stod(joint.parameters.at("external_gear_ratio"));
      }
      else
      {
        external_gear_ratios_[i] = 1.0;
      }

      if (joint.parameters.count("acc_limit") != 0 &&
        joint.parameters.count("vel_limit") != 0)
      {
        std::pair<std::int32_t, std::int32_t> limits;
        limits.first = std::stoi(joint.parameters.at("vel_limit")) / 10 * erpm_conversion;
        limits.second = std::stoi(joint.parameters.at("acc_limit")) / 10 * erpm_conversion;
        if (limits.first >= 32767 || limits.first <= 0)
        {
          RCLCPP_ERROR(
            rclcpp::get_logger("CubeMarsSystemHardware"),
            "velocity limit is not in range 0-32767: %d", limits.first);
          return hardware_interface::CallbackReturn::ERROR;
        }
        if (limits.second >= 32767 || limits.second <= 0)
        {
          RCLCPP_ERROR(
            rclcpp::get_logger("CubeMarsSystemHardware"),
            "acceleration limit is not in range 0-32767: %d", limits.second);
          return hardware_interface::CallbackReturn::ERROR;
        }
        limits_.emplace_back(limits);
      }
      else
      {
        limits_.emplace_back(std::make_pair(0, 0));
      }
    }
    else
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("CubeMarsSystemHardware"),
        "Missing parameters in URDF for %s", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.parameters.count("enc_off") != 0)
    {
      enc_offs_.emplace_back(std::stod(joint.parameters.at("enc_off")));
    }
    else
    {
      enc_offs_.emplace_back(0);
    }

    if (joint.parameters.count("trq_limit") != 0 && std::stod(joint.parameters.at("trq_limit")) > 0)
    {
      trq_limits_.emplace_back(std::stod(joint.parameters.at("trq_limit")));
    }
    else
    {
      trq_limits_.emplace_back(0);
    }

    if (joint.parameters.count("read_only") != 0 && std::stoi(joint.parameters.at("read_only")) == 1)
    {
      read_only_.emplace_back(true);
    }
    else
    {
      read_only_.emplace_back(false);
    }

    // Check if this joint has grasp detection enabled (for gripper)
    if (joint.parameters.count("grasp_detect") != 0 && std::stoi(joint.parameters.at("grasp_detect")) == 1)
    {
      grasp_detect_[i] = true;
      RCLCPP_INFO(
        rclcpp::get_logger("CubeMarsSystemHardware"),
        "Grasp detection enabled for joint %s with torque limit %.3f Nm", 
        joint.name.c_str(), trq_limits_[i]);
    }
    else
    {
      grasp_detect_[i] = false;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CubeMarsSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const hardware_interface::CallbackReturn result = can_.connect(can_itf_, can_ids_, 0xFFU)
                                                      ? hardware_interface::CallbackReturn::SUCCESS
                                                      : hardware_interface::CallbackReturn::FAILURE;

  RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "Communication active");

  return result;
}

hardware_interface::CallbackReturn CubeMarsSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const hardware_interface::CallbackReturn result = can_.disconnect()
                                                      ? hardware_interface::CallbackReturn::SUCCESS
                                                      : hardware_interface::CallbackReturn::FAILURE;

  RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "Communication closed");

  return result;
}

std::vector<hardware_interface::StateInterface>
CubeMarsSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_efforts_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "temperature", &hw_states_temperatures_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
CubeMarsSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_commands_accelerations_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_efforts_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type CubeMarsSystemHardware::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  stop_modes_.clear();
  start_modes_.clear();

  stop_modes_.resize(info_.joints.size(), false);

  // Define allowed combination of command interfaces
  std::unordered_set<std::string> eff {"effort"};
  std::unordered_set<std::string> vel {"velocity"};
  std::unordered_set<std::string> pos {"position"};
  
  std::unordered_set<std::string> joint_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    // find stop modes
    for (std::string key : stop_interfaces)
    {
      RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "stop interface: %s", key.c_str());
      if (key.find(info_.joints[i].name) != std::string::npos)
      {
        stop_modes_[i] = true;
        break;
      }
    }

    // find start modes
    joint_interfaces.clear();
    for (std::string key : start_interfaces)
    {
      RCLCPP_INFO(rclcpp::get_logger("CubeMarsSystemHardware"), "start interface: %s", key.c_str());
      if (key.find(info_.joints[i].name) != std::string::npos)
      {
        joint_interfaces.insert(key.substr(key.find("/") + 1));
      }
    }
    if (joint_interfaces == eff)
    {
      start_modes_.push_back(CURRENT_LOOP);
    }
    else if (joint_interfaces == vel)
    {
      start_modes_.push_back(SPEED_LOOP);
    }
    else if (joint_interfaces == pos)
    {
      if (limits_[i].first == 0 || limits_[i].second == 0)
      {
        start_modes_.push_back(POSITION_LOOP);
      }
      else
      {
        start_modes_.push_back(POSITION_SPEED_LOOP);
      }
    }
    else if (joint_interfaces.empty())
    {
      if (stop_modes_[i])
      {
        start_modes_.push_back(UNDEFINED);
      }
      else
      {
        // don't change control mode
        start_modes_.push_back(control_mode_[i]);
      }
    }
    else
    {
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CubeMarsSystemHardware::perform_command_mode_switch(
  const std::vector<std::string> & /*start_interfaces*/,
  const std::vector<std::string> & /*stop_interfaces*/)
{
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    if (stop_modes_[i])
    {
      hw_commands_efforts_[i] = std::numeric_limits<double>::quiet_NaN();
      hw_commands_velocities_[i] = std::numeric_limits<double>::quiet_NaN();
      hw_commands_positions_[i] = std::numeric_limits<double>::quiet_NaN();
      // Reset grasp detection when stopping
      grasp_detected_[i] = false;
      target_position_[i] = std::numeric_limits<double>::quiet_NaN();
    }
    // switch control mode
    control_mode_[i] = start_modes_[i];
  }
  return hardware_interface::return_type::OK;
}


hardware_interface::CallbackReturn CubeMarsSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CubeMarsSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type CubeMarsSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  bool all_ids[can_ids_.size()] = { false };
  std::uint32_t read_id;
  std::uint8_t read_data[8];
  std::uint8_t read_len;

  std::int16_t pos_int;

  // read all buffered CAN messages
  while (can_.read_nonblocking(read_id, read_data, read_len))
  {
    if (read_data[7] != 0)
    {
      switch(read_data[7])
      {
        case 1:
          RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "Motor over-temperature fault.");
          break;
        case 2:
          RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "Over-current fault.");
          break;
        case 3:
          RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "Over-voltage fault.");
          break;
        case 4:
          RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "Under-voltage fault.");
          break;
        case 5:
          RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "Encoder fault.");
          break;
        case 6:
          RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "MOSFET over-temperature fault.");
          break;
        case 7:
          RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"), "Motor stall.");
          break;
        return hardware_interface::return_type::ERROR;
      }
    }
    auto it = std::find(can_ids_.begin(), can_ids_.end(), read_id);
    if (it != can_ids_.end())
    {
      int i = std::distance(can_ids_.begin(), it);
      all_ids[i] = true;
      pos_int = read_data[0] << 8 | read_data[1];
      hw_states_positions_[i] = pos_int;
      hw_states_velocities_[i] = std::int16_t (read_data[2] << 8 | read_data[3]);
      hw_states_efforts_[i] = std::int16_t (read_data[4] << 8 | read_data[5]);
    }
  }

  // check if all CAN IDs have received a message
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    if (!all_ids[i])
    {
      RCLCPP_WARN(
        rclcpp::get_logger("CubeMarsSystemHardware"),
        "No CAN message received from CAN ID: %u. ",
        can_ids_[i]);
    }
    else
    {
      // Unit conversions
      hw_states_positions_[i] = (hw_states_positions_[i] * 0.1 * M_PI / 180) / external_gear_ratios_[i] - enc_offs_[i];
      hw_states_velocities_[i] = (hw_states_velocities_[i] * 10 / erpm_conversions_[i]) / external_gear_ratios_[i];
      hw_states_efforts_[i] = hw_states_efforts_[i] * 0.01 * torque_constants_[i] *
        internal_gear_ratios_[i] * external_gear_ratios_[i];
      hw_states_temperatures_[i] = read_data[6];
      
      // Grasp detection for gripper
      if (grasp_detect_[i] && trq_limits_[i] > 0)
      {
        // Check if torque exceeds threshold
        if (std::abs(hw_states_efforts_[i]) >= trq_limits_[i])
        {
          if (!grasp_detected_[i])
          {
            RCLCPP_INFO(
              rclcpp::get_logger("CubeMarsSystemHardware"),
              "Grasp detected on joint %lu! Torque: %.3f Nm (limit: %.3f Nm)", 
              i, hw_states_efforts_[i], trq_limits_[i]);
            grasp_detected_[i] = true;
            // Store current position as the grasp position
            target_position_[i] = hw_states_positions_[i];
          }
        }
        else
        {
          // Reset grasp detection if torque drops below threshold
          // Use 0.5 (50%) for worm gear - balance between stability and release
          if (grasp_detected_[i] && std::abs(hw_states_efforts_[i]) < trq_limits_[i] * 0.9)
          {
            RCLCPP_INFO(
              rclcpp::get_logger("CubeMarsSystemHardware"),
              "Grasp released on joint %lu (torque: %.3f Nm)", i, hw_states_efforts_[i]);
            grasp_detected_[i] = false;
            target_position_[i] = std::numeric_limits<double>::quiet_NaN();
          }
        }
        // Note: Do NOT return ERROR for grasp detection - this is normal operation
      }
      // Regular torque limit check for non-gripper joints (safety shutdown)
      else if (trq_limits_[i] != 0 && std::abs(hw_states_efforts_[i]) > trq_limits_[i])
      {
        RCLCPP_ERROR(rclcpp::get_logger("CubeMarsSystemHardware"),
          "Joint %lu went over torque limit - SAFETY SHUTDOWN!", i);
        
        // disable motor
        std::uint8_t data[4] = {0, 0, 0, 0};
        can_.write_message(can_ids_[i] | CURRENT_LOOP << 8, data, 4);
        
        return hardware_interface::return_type::ERROR;
      }
      
      if (read_only_[i])
      {
        RCLCPP_INFO(
          rclcpp::get_logger("CubeMarsSystemHardware"),
          "Joint %lu: pos: %f",
          i, hw_states_positions_[i]);
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CubeMarsSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    if (!read_only_[i])
    {
      switch (control_mode_[i])
      {
        case UNDEFINED:
        {
            break;
        }
        case CURRENT_LOOP:
        {
          if (!std::isnan(hw_commands_efforts_[i]))
          {
            // For gripper with grasp detection, zero current when grasp is detected
            double command_effort = hw_commands_efforts_[i];
            if (grasp_detect_[i] && grasp_detected_[i])
            {
              // Set current to zero to stop applying force
              command_effort = 0.0;
              RCLCPP_INFO_THROTTLE(
                rclcpp::get_logger("CubeMarsSystemHardware"),
                steady_clock,
                1000,  // Log every 1 second
                "Gripper joint %lu holding grasp - zeroing current (commanded effort: %.3f Nm)", 
                i, hw_commands_efforts_[i]);
            }
            
            std::int32_t current = (command_effort / (internal_gear_ratios_[i] * external_gear_ratios_[i])) * 1000 / torque_constants_[i];
            if (std::abs(current) >= 60000)
            {
              RCLCPP_ERROR(
                rclcpp::get_logger("CubeMarsSystemHardware"),
                "current command is over maximal allowed value of 60000: %d", current);
              return hardware_interface::return_type::ERROR;
            }

            std::uint8_t data[4];
            data[0] = current >> 24;
            data[1] = current >> 16;
            data[2] = current >> 8;
            data[3] = current;

            can_.write_message(can_ids_[i] | CURRENT_LOOP << 8, data, 4);
          }
          break;
        }
        case SPEED_LOOP:
        {
          if (!std::isnan(hw_commands_velocities_[i]))
          {
            // For gripper with grasp detection, zero velocity when grasp is detected
            double command_velocity = hw_commands_velocities_[i];
            if (grasp_detect_[i] && grasp_detected_[i])
            {
              // Set velocity to zero to stop moving
              command_velocity = 0.0;
              RCLCPP_INFO_THROTTLE(
                rclcpp::get_logger("CubeMarsSystemHardware"),
                steady_clock,
                1000,  // Log every 1 second
                "Gripper joint %lu holding grasp - zeroing velocity (commanded: %.3f rad/s)", 
                i, hw_commands_velocities_[i]);
            }
            
            std::int32_t speed = (command_velocity * external_gear_ratios_[i]) * erpm_conversions_[i];
            if (std::abs(speed) >= 100000)
            {
              RCLCPP_ERROR(
                rclcpp::get_logger("CubeMarsSystemHardware"),
                "speed command is over maximal allowed value of 100000: %d", speed);
              return hardware_interface::return_type::ERROR;
            }

            std::uint8_t data[4];
            data[0] = speed >> 24;
            data[1] = speed >> 16;
            data[2] = speed >> 8;
            data[3] = speed;

            can_.write_message(can_ids_[i] | SPEED_LOOP << 8, data, 4);
          }
          break;
        }
        case POSITION_LOOP:
        {
          if (!std::isnan(hw_commands_positions_[i]))
          {
            // For gripper with grasp detection, hold position if grasp is detected
            double command_position = hw_commands_positions_[i];
            if (grasp_detect_[i] && grasp_detected_[i])
            {
              // Send CURRENT position (not commanded) to stop motor from fighting
              // This prevents PID oscillation on worm gear grippers
              command_position = target_position_[i];
              
              RCLCPP_INFO_THROTTLE(
                rclcpp::get_logger("CubeMarsSystemHardware"),
                steady_clock,
                1000,  // Log every 1 second
                "Gripper joint %lu holding grasp: pos=%.3f (commanded=%.3f), torque=%.3f Nm", 
                i, command_position, hw_commands_positions_[i], hw_states_efforts_[i]);
            }
            
            std::int32_t position = ((command_position + enc_offs_[i]) * external_gear_ratios_[i]) * 10000 * 180 / M_PI;
            if (std::abs(position) >= 360000000)
            {
              RCLCPP_ERROR(
                rclcpp::get_logger("CubeMarsSystemHardware"),
                "position command is over maximal allowed value of 360000000: %d", position);
              return hardware_interface::return_type::ERROR;
            }

            std::uint8_t data[4];
            data[0] = position >> 24;
            data[1] = position >> 16;
            data[2] = position >> 8;
            data[3] = position;

            can_.write_message(can_ids_[i] | POSITION_LOOP << 8, data, 4);
          }
          break;
        }
        case POSITION_SPEED_LOOP:
        {
          if (!std::isnan(hw_commands_positions_[i]))
          {
            // For gripper with grasp detection, hold position if grasp is detected
            double command_position = hw_commands_positions_[i];
            if (grasp_detect_[i] && grasp_detected_[i])
            {
              // Send CURRENT position (not commanded) to stop motor from fighting
              // This prevents PID oscillation on worm gear grippers
              command_position = target_position_[i];
              
              RCLCPP_INFO_THROTTLE(
                rclcpp::get_logger("CubeMarsSystemHardware"),
                steady_clock,
                1000,  // Log every 1 second
                "Gripper joint %lu holding grasp: pos=%.3f (commanded=%.3f), torque=%.3f Nm", 
                i, command_position, hw_commands_positions_[i], hw_states_efforts_[i]);
            }
            
            std::int32_t position = ((command_position + enc_offs_[i]) * external_gear_ratios_[i]) * 10000 * 180 / M_PI;
            std::int16_t vel = limits_[i].first;
            std::int16_t acc = limits_[i].second;
            if (std::abs(position) >= 360000000)
            {
              RCLCPP_ERROR(
                rclcpp::get_logger("CubeMarsSystemHardware"),
                "position command is over maximal allowed value of 360000000: %d", position);
              return hardware_interface::return_type::ERROR;
            }

            std::uint8_t data[8];
            data[0] = position >> 24;
            data[1] = position >> 16;
            data[2] = position >> 8;
            data[3] = position;
            data[4] = vel >> 8;
            data[5] = vel;
            data[6] = acc >> 8;
            data[7] = acc;

            can_.write_message(can_ids_[i] | POSITION_SPEED_LOOP << 8, data, 8);
          }
          break;
        }
      }
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace cubemars_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  cubemars_hardware::CubeMarsSystemHardware, hardware_interface::SystemInterface)