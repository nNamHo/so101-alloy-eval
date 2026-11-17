#include <fmt/ranges.h>

#include <sstream>
#include <iomanip>
#include <thread>
#include <chrono>

#include <algorithm>
#include <feetech_driver/common.hpp>
#include <feetech_driver/communication_protocol.hpp>
#include <feetech_ros2_driver/feetech_ros2_driver.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/all.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

namespace feetech_ros2_driver {
#if HARDWARE_INTERFACE_VERSION_GTE(4, 34, 0)
CallbackReturn FeetechHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams& params) {
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
#else
CallbackReturn FeetechHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
#endif
    return CallbackReturn::ERROR;
  }

  if (init_transport_() != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  JointIdConfigMap yaml_by_id;
  if (load_yaml_config_and_warn_(yaml_by_id) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  if (configure_joints_(yaml_by_id) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  if (validate_model_series_() != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn FeetechHardwareInterface::init_transport_() {
  const auto usb_port_it = info_.hardware_parameters.find("usb_port");
  if (usb_port_it == info_.hardware_parameters.end()) {
    spdlog::error(
        "FeetechHardwareInterface::init_transport_ Hardware parameter [usb_port] not found! "
        "Make sure to have <param name=\"usb_port\">/dev/XXXX</param>");
    return CallbackReturn::ERROR;
  }

  auto serial_port = std::make_unique<feetech_driver::SerialPort>(usb_port_it->second);

  if (const auto result = serial_port->configure(); !result) {
    spdlog::error("FeetechHardwareInterface::init_transport_ -> {}", result.error());
    return CallbackReturn::ERROR;
  }

  communication_protocol_ = std::make_unique<feetech_driver::CommunicationProtocol>(std::move(serial_port));

  return CallbackReturn::SUCCESS;
}

// Optional YAML overlay — if not provided, URDF params are used as-is.
// Builds an ID-keyed map: URDF id is the hardware identity, YAML name is just a label.
CallbackReturn FeetechHardwareInterface::load_yaml_config_and_warn_(JointIdConfigMap& out_yaml) {
  out_yaml.clear();

  const auto cfg_it = info_.hardware_parameters.find("joint_config_file");
  if (cfg_it == info_.hardware_parameters.end() || cfg_it->second.empty()) {
    return CallbackReturn::SUCCESS;  // no YAML — fall back to URDF params only
  }

  auto loaded = load_joint_config(cfg_it->second);
  if (!loaded) {
    return CallbackReturn::ERROR;
  }

  // Re-key by servo id
  for (auto& [name, params] : *loaded) {
    auto it = params.find("id");
    if (it == params.end()) {
      spdlog::error("YAML joint '{}' has no 'id' parameter", name);
      return CallbackReturn::ERROR;
    }
    int id = std::stoi(it->second);
    if (!out_yaml.emplace(id, std::move(params)).second) {
      spdlog::error("Duplicate servo id {} in YAML (joint '{}')", id, name);
      return CallbackReturn::ERROR;
    }
  }

  // Warn: URDF ids missing in YAML
  for (const auto& j : info_.joints) {
    auto id_it = j.parameters.find("id");
    if (id_it != j.parameters.end() && out_yaml.find(std::stoi(id_it->second)) == out_yaml.end()) {
      spdlog::warn("URDF joint '{}' (id={}) has no YAML entry (using URDF defaults)", j.name, id_it->second);
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn FeetechHardwareInterface::configure_joints_(const JointIdConfigMap& yaml_by_id) {
  joint_ids_.assign(info_.joints.size(), 0);

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto& joint = info_.joints[i];
    const std::string& joint_name = joint.name;

    // Required: id (from URDF — hardware identity)
    const auto urdf_id_it = joint.parameters.find("id");
    if (urdf_id_it == joint.parameters.end()) {
      spdlog::error("Joint '{}' does not have required 'id' parameter", joint_name);
      return CallbackReturn::ERROR;
    }
    const int id = std::stoi(urdf_id_it->second);
    joint_ids_[i] = static_cast<uint8_t>(id);

    // Merge YAML config (looked up by servo id) over URDF params
    JointParams merged_params;
    if (auto it = yaml_by_id.find(id); it != yaml_by_id.end()) {
      merged_params = merge_joint_params(it->second, joint.parameters);
    } else {
      merged_params = JointParams(joint.parameters.begin(), joint.parameters.end());
    }

    if (merged_params.find("offset") != merged_params.end()) {
      spdlog::warn("Joint '{}': 'offset' param is deprecated and ignored — use 'homing_offset' instead", joint_name);
    }

    // Disable torque and unlock EPROM before writing parameters
    if (const auto result = communication_protocol_->disable_torque(joint_ids_[i]); !result) {
      spdlog::error("FeetechHardwareInterface::configure_joints_ disable_torque -> {}", result.error());
      return CallbackReturn::ERROR;
    }

    // Single-byte parameters (0-255)
    for (const auto& [parameter_name, address] : {std::pair{"p_coefficient", SMS_STS_P_COEF},
                                                  {"d_coefficient", SMS_STS_D_COEF},
                                                  {"i_coefficient", SMS_STS_I_COEF},
                                                  {"overload_torque", SMS_STS_OVERLOAD_TORQUE},
                                                  {"return_delay_time", SMS_STS_RETURN_DELAY},
                                                  {"acceleration", SMS_STS_ACC}}) {
      if (const auto param_it = merged_params.find(parameter_name); param_it != merged_params.end()) {
        const auto result = communication_protocol_->write(
            joint_ids_[i], address, std::experimental::make_array(static_cast<uint8_t>(std::stoi(param_it->second))));
        if (!result) {
          spdlog::error("FeetechHardwareInterface::configure_joints_ -> {}", result.error());
          return CallbackReturn::ERROR;
        }
      }
    }

    // Two-byte unsigned parameters
    for (const auto& [parameter_name, address] : {std::pair{"range_min", SMS_STS_MIN_ANGLE_LIMIT_L},
                                                  {"range_max", SMS_STS_MAX_ANGLE_LIMIT_L},
                                                  {"max_torque_limit", SMS_STS_MAX_TORQUE_L},
                                                  {"protection_current", SMS_STS_PROTECTION_CURRENT_L}}) {
      if (const auto param_it = merged_params.find(parameter_name); param_it != merged_params.end()) {
        std::array<uint8_t, 2> buf{};
        feetech_driver::to_sts(&buf[0], &buf[1], std::stoi(param_it->second));
        const auto result = communication_protocol_->write(joint_ids_[i], address, buf);
        if (!result) {
          spdlog::error("FeetechHardwareInterface::configure_joints_ -> {}", result.error());
          return CallbackReturn::ERROR;
        }
      }
    }

    // Two-byte signed parameters (sign-magnitude encoding)
    for (const auto& [parameter_name, address, sign_bit] :
         {std::tuple{"homing_offset", SMS_STS_OFS_L, SMS_STS_SIGN_BIT_HOMING_OFFSET}}) {
      if (const auto param_it = merged_params.find(parameter_name); param_it != merged_params.end()) {
        std::array<uint8_t, 2> buf{};
        const int value = feetech_driver::encode_sign_magnitude(std::stoi(param_it->second), sign_bit);
        feetech_driver::to_sts(&buf[0], &buf[1], value);
        const auto result = communication_protocol_->write(joint_ids_[i], address, buf);
        if (!result) {
          spdlog::error("FeetechHardwareInterface::configure_joints_ -> {}", result.error());
          return CallbackReturn::ERROR;
        }
      }
    }

    // Lock EPROM after writing parameters (for all joints)
    if (const auto result = communication_protocol_->lock_eprom(joint_ids_[i]); !result) {
      spdlog::error("FeetechHardwareInterface::configure_joints_ lock_eprom -> {}", result.error());
      return CallbackReturn::ERROR;
    }

    // Torque enable MOVED to on_activate. Enabling here drives each servo to
    // whatever stale goal_position its RAM holds (typically the 2048 power-on
    // default = the calibration pose) before anything seeds that register.
    // Correct order is: read present -> write goal = present -> enable torque.
    //
    // if (!joint.command_interfaces.empty()) {
    //   if (const auto result = communication_protocol_->set_torque(joint_ids_[i], true); !result) {
    //     spdlog::error("FeetechHardwareInterface::configure_joints_ set_torque -> {}", result.error());
    //     return CallbackReturn::ERROR;
    //   }
    // }
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn FeetechHardwareInterface::validate_model_series_() {
  const auto joint_model_series = joint_ids_ | ranges::views::transform([&](const auto id) {
                                    return communication_protocol_->read_model_number(id)
                                        .and_then(feetech_driver::get_model_name)
                                        .and_then(feetech_driver::get_model_series);
                                  });

  if (std::ranges::any_of(joint_model_series, [](const auto& series) { return !series.has_value(); })) {
    spdlog::error("FeetechHardware::validate_model_series_ [One of the joints has an error]. Input: {}",
                  ranges::views::zip(joint_ids_, joint_model_series));
    return CallbackReturn::ERROR;
  }

  const auto js = joint_model_series | ranges::views::transform([](const auto& series) { return series.value(); });

  if (ranges::any_of(js, [](const auto& series) { return series != feetech_driver::ModelSeries::kSts; })) {
    spdlog::error("FeetechHardware::validate_model_series_ [Only STS series is supported]. Input (id, series): {}",
                  ranges::views::zip(joint_ids_, js));
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FeetechHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_hw_positions_.resize(info_.joints.size(), 0.0);
  state_hw_velocities_.resize(info_.joints.size(), 0.0);
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &state_hw_positions_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &state_hw_velocities_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FeetechHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  for (uint i = 0; i < info_.joints.size(); i++) {
    if (!info_.joints[i].command_interfaces.empty()) {
      command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    }
  }

  return command_interfaces;
}

hardware_interface::return_type FeetechHardwareInterface::read(const rclcpp::Time&, const rclcpp::Duration&) {
  constexpr int kMaxRetries = 3;
  for (size_t i = 0; i < joint_ids_.size(); ++i) {
    feetech_driver::Expected<int> pos;
    int attempt = 0;
    do {
      pos = communication_protocol_->read_word(joint_ids_[i], SMS_STS_PRESENT_POSITION_L);
      if (pos) break;
      ++attempt;
    } while (attempt < kMaxRetries);

    if (!pos) {
      // Keep the last known good value rather than killing the whole stack.
      spdlog::warn("read [id={}] failed after {} retries; holding last value",
                   joint_ids_[i], kMaxRetries);
      continue;
    }
    state_hw_positions_[i] = feetech_driver::to_radians(*pos - feetech_driver::kStsMidpoint);
    state_hw_velocities_[i] = 0.0;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FeetechHardwareInterface::write(const rclcpp::Time& /* time */,
                                                                const rclcpp::Duration& /* period */) {
  // Create vectors only for joints that have command interfaces
  std::vector<uint8_t> commanded_joint_ids;
  std::vector<int> commanded_positions;
  std::vector<int> commanded_speeds;
  std::vector<int> commanded_accelerations;

  for (uint i = 0; i < info_.joints.size(); i++) {
    // Only include joints with command interfaces
    if (!info_.joints[i].command_interfaces.empty()) {
      commanded_joint_ids.push_back(joint_ids_[i]);
      commanded_positions.push_back(feetech_driver::from_radians(hw_positions_[i]) + feetech_driver::kStsMidpoint);
      commanded_speeds.push_back(2400);       // Default speed
      commanded_accelerations.push_back(50);  // Default acceleration
    }
  }

  // Only send commands if there are joints to command
  if (!commanded_joint_ids.empty()) {

    // Diagnostic: log only when the command CHANGES, so trajectory execution is
    // visible no matter how late it starts (a fixed first-N logger expires before
    // the commander even plans). CMD CHANGE #0 is the activation seed; a stream
    // of changes during a trajectory proves the controller's output reaches the
    // bus, and their values show exactly what the servos were told to do.
    static std::vector<double> last_cmd;
    static int chg = 0;
    bool changed = (last_cmd.size() != hw_positions_.size());
    if (!changed) {
      for (size_t i = 0; i < hw_positions_.size(); ++i) {
        if (std::abs(hw_positions_[i] - last_cmd[i]) > 1e-6) { changed = true; break; }
      }
    }
    if (changed && chg < 200) {
      std::ostringstream oss;
      oss << "CMD CHANGE #" << chg << ":";
      for (size_t i = 0; i < commanded_joint_ids.size(); ++i)
        oss << " id" << static_cast<int>(commanded_joint_ids[i]) << "="
            << commanded_positions[i];
      oss << "  | rad:";
      for (size_t i = 0; i < hw_positions_.size(); ++i)
        oss << " " << std::fixed << std::setprecision(4) << hw_positions_[i];
      spdlog::info(oss.str());
      last_cmd = hw_positions_;
      ++chg;
    }

    const auto write_result = communication_protocol_->sync_write_position(
        commanded_joint_ids, commanded_positions, commanded_speeds, commanded_accelerations);
    if (!write_result) {
      spdlog::error("FeetechHardwareInterface::write -> {}", write_result.error());
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}


CallbackReturn FeetechHardwareInterface::on_activate(const rclcpp_lifecycle::State&) {
  constexpr int kSeedRetries = 10;   // generous: this runs once, latency doesn't matter

  for (size_t i = 0; i < joint_ids_.size(); ++i) {
    feetech_driver::Expected<int> pos;
    for (int attempt = 0; attempt < kSeedRetries; ++attempt) {
      pos = communication_protocol_->read_word(joint_ids_[i], SMS_STS_PRESENT_POSITION_L);
      if (pos) break;
      spdlog::warn("on_activate seed read [id={}] attempt {}/{} failed",
                   joint_ids_[i], attempt + 1, kSeedRetries);
    }
    if (!pos) {
      // Fatal on purpose: an unseeded joint becomes a command of 0.0 rad and the
      // arm lurches to the calibration pose. Fail loudly instead.
      spdlog::error("on_activate: could not seed joint id={} after {} attempts — aborting activation",
                    joint_ids_[i], kSeedRetries);
      return CallbackReturn::ERROR;
    }
    state_hw_positions_[i] = feetech_driver::to_radians(*pos - feetech_driver::kStsMidpoint);
    state_hw_velocities_[i] = 0.0;
    spdlog::info("on_activate seeded joint id={} at {:.4f} rad",
                 joint_ids_[i], state_hw_positions_[i]);
  }

  hw_positions_ = state_hw_positions_;   // command starts where the arm actually is

  // Seed each servo's goal_position register while torque is still OFF, so
  // enabling torque below is a no-op rather than a lurch to the stale goal.
  write(rclcpp::Time{}, rclcpp::Duration::from_seconds(0));


  // Let the bus settle after the sync_write burst. On this platform libserial
  // drops reads that follow writes too closely (same fault class as sync_read).
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  std::vector<uint8_t> torque_enabled;
  for (size_t i = 0; i < joint_ids_.size(); ++i) {
    if (info_.joints[i].command_interfaces.empty()) continue;   // read-only joints

    bool ok = false;
    for (int attempt = 0; attempt < kSeedRetries; ++attempt) {
      const auto result = communication_protocol_->set_torque(joint_ids_[i], true);
      if (result) { ok = true; break; }
      spdlog::warn("on_activate set_torque [id={}] attempt {}/{} -> {}",
                   joint_ids_[i], attempt + 1, kSeedRetries, result.error());
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    if (!ok) {
      spdlog::error("on_activate: set_torque failed for id={} after {} attempts",
                    joint_ids_[i], kSeedRetries);
      // on_deactivate will NOT run after we return ERROR, and a timed-out
      // set_torque may still have reached the servo. Release everything.
      for (const auto id : torque_enabled) {
        if (const auto r = communication_protocol_->set_torque(id, false); !r) {
          spdlog::error("on_activate cleanup: failed to release torque id={} -> {}", id, r.error());
        }
      }
      if (const auto r = communication_protocol_->set_torque(joint_ids_[i], false); !r) {
        spdlog::error("on_activate cleanup: failed to release torque id={} -> {}",
                      joint_ids_[i], r.error());
      }
      return CallbackReturn::ERROR;
    }

    torque_enabled.push_back(joint_ids_[i]);
    spdlog::info("on_activate torque ON for id={} holding {:.4f} rad",
                 joint_ids_[i], hw_positions_[i]);

  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn FeetechHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /* previous_state */) {
  // all joints torque off
  const auto torque_disable_parameters =
      std::vector(joint_ids_.size(), std::experimental::make_array(static_cast<uint8_t>(0)));
  if (const auto result =
          communication_protocol_->sync_write(joint_ids_, SMS_STS_TORQUE_ENABLE, torque_disable_parameters);
      !result) {
    spdlog::error("FeetechHardwareInterface::on_deactivate -> {}", result.error());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}



}  // namespace feetech_ros2_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(feetech_ros2_driver::FeetechHardwareInterface, hardware_interface::SystemInterface)
