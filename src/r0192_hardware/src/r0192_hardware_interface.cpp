#include "r0192_hardware/r0192_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>

namespace r0192_hardware
{

hardware_interface::CallbackReturn R0192SystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_efforts_.resize(info_.joints.size(), 0.0);

  hw_cmd_positions_.resize(info_.joints.size(), 0.0);
  hw_cmd_velocities_.resize(info_.joints.size(), 0.0);
  hw_cmd_efforts_.resize(info_.joints.size(), 0.0);
  hw_cmd_kp_.resize(info_.joints.size(), 0.0);
  hw_cmd_kd_.resize(info_.joints.size(), 0.0);

  for (size_t i = 0; i < info_.joints.size(); i++) {
    joint_index_[info_.joints[i].name] = i;
    const auto & params = info_.joints[i].parameters;
    if (params.count("kp")) hw_cmd_kp_[i] = std::stod(params.at("kp"));
    if (params.count("kd")) hw_cmd_kd_[i] = std::stod(params.at("kd"));
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn R0192SystemHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = rclcpp::get_logger("R0192Hardware");

  can_comm_ = std::make_shared<CanCommunication>("can0");
  can_available_ = can_comm_->init();

  if (!can_available_) {
    RCLCPP_WARN(logger,
      "CAN socket 'can0' not available — running in virtual (passthrough) mode. "
      "To enable real motors run: sudo ip link set can0 up type can bitrate 500000");
  } else {
    axis1_ = std::make_shared<GDS68Driver>(0x01, can_comm_, logger);
    axis4_ = std::make_shared<RS05Driver>(0x04, can_comm_, logger);
    RCLCPP_INFO(logger, "R0192 hardware configured (CAN 'can0' open)");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn R0192SystemHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = rclcpp::get_logger("R0192Hardware");

  if (can_available_) {
    rx_thread_running_ = true;
    rx_thread_ = std::thread(&R0192SystemHardware::canRxThread, this);

    // Enable closed-loop first.
    axis1_->Set_Axis_State(8);
    axis4_->Motor_Enabled_To_Run();

    // Set GDS68 to Position Control (3) and Passthrough (1) to prevent drift.
    // This uses the motor's robust internal PID instead of the external MIT_Control.
    axis1_->Set_Controller_Mode(3, 1);

    // GDS68 (axis 1): zero the encoder at current physical position so that
    // MIT_Control commands stay within ±12.5 rad.  After Set_Linear_Count(0)
    // the motor reports position ≈ 0 and MIT_Control(0.0) holds it in place.
    if (joint_index_.count("joint_1")) {
      const size_t i = joint_index_.at("joint_1");
      // NOTE: GDS68 bootsequent & homing implementing later!

      axis1_->Get_Encoder_Estimates();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      RCLCPP_INFO(logger, "Axis 1 raw encoder: %.3f rad — zeroing at current position",
                  axis1_->get_current_position());
      
      // Set motor mechanical zero
      axis1_->Set_Linear_Count(0);
      RCLCPP_INFO(logger, "Axis 1 mechanical zero set");

      // Wait for the autonomous encoder update (10 ms interval) to reflect the new zero.
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      axis1_->Get_Encoder_Estimates();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));

      double pos = axis1_->get_current_position();
      RCLCPP_INFO(logger, "Axis 1 after zero: %.3f rad (motor holds current physical position)", pos);
      hw_cmd_positions_[i] = pos;
    }

    // RS05 (axis 4): sends feedback autonomously — wait briefly then home.
    if (joint_index_.count("joint_4")) {
      const size_t i = joint_index_.at("joint_4");
      // NOTE: RS05 bootsequent & homing implementing later!
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      RCLCPP_INFO(logger, "Axis 4 raw encoder: %.3f rad — zeroing at current position", axis4_->get_current_position());

      // Set motor mechanical zero
      axis4_->Set_Motor_Mechanical_Zero();
      RCLCPP_INFO(logger, "Axis 4 mechanical zero set");

      std::this_thread::sleep_for(std::chrono::milliseconds(50));

      double pos = axis4_->get_current_position();
      RCLCPP_INFO(logger, "Axis 4 after zero: %.3f rad", pos);
      hw_cmd_positions_[i] = pos;
    }
  } else {
    RCLCPP_WARN(logger, "Activated in virtual mode — no CAN frames will be sent or received");
  }

  RCLCPP_INFO(logger, "R0192 hardware activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn R0192SystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = rclcpp::get_logger("R0192Hardware");
  if (can_available_) {

    axis1_->Set_Axis_State(1);
    axis4_->Motor_Stop_Running();

    rx_thread_running_ = false;
    if (rx_thread_.joinable()) {
      rx_thread_.join();
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("R0192Hardware"), "R0192 hardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> R0192SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT,   &hw_efforts_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> R0192SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_cmd_positions_[i]);
    command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_cmd_velocities_[i]);
    command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT,   &hw_cmd_efforts_[i]);
    command_interfaces.emplace_back(info_.joints[i].name, "kp", &hw_cmd_kp_[i]);
    command_interfaces.emplace_back(info_.joints[i].name, "kd", &hw_cmd_kd_[i]);
  }
  return command_interfaces;
}

// ==============================================================================
// read(): Ist-Werte abholen
//   - joint_1, joint_4: echte Motorrückmeldung via CAN-Treiber
//   - joint_2/3/5/6/7:  Passthrough (Soll = Ist) → perfekte Fake-Rückmeldung
//                        für Achsen ohne physischen Motor
//   - joint_8:           Mimic von joint_7 (Multiplikator -1)
// ==============================================================================
hardware_interface::return_type R0192SystemHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // --- Achsen mit echtem Motor (nur wenn CAN verfügbar) ---
  if (can_available_) {
    if (joint_index_.count("joint_1")) {
      const size_t i = joint_index_.at("joint_1");
      hw_positions_[i] = axis1_->get_current_position();
      hw_velocities_[i] = axis1_->get_current_velocity();
      hw_efforts_[i]    = axis1_->get_current_torque();
      // GDS68 does not push feedback autonomously — request every cycle.
      // Reply is processed asynchronously by canRxThread and available next cycle.
      // axis1_->Get_Encoder_Estimates();
    }
    if (joint_index_.count("joint_4")) {
      const size_t i = joint_index_.at("joint_4");
      hw_positions_[i] = axis4_->get_current_position();
      hw_velocities_[i] = axis4_->get_current_velocity();
      hw_efforts_[i]    = axis4_->get_current_torque();
      // RS05 sends feedback autonomously — no request needed.
    }
  } else {
    // Virtual mode: axes 1 and 4 also use passthrough
    for (const auto& jname : {"joint_1", "joint_4"}) {
      if (joint_index_.count(jname)) {
        const size_t i = joint_index_.at(jname);
        hw_positions_[i]  = hw_cmd_positions_[i];
        hw_velocities_[i] = hw_cmd_velocities_[i];
        hw_efforts_[i]    = 0.0;
      }
    }
  }

  // --- Passthrough für Achsen ohne Motor ---
  // Soll-Wert des letzten write()-Zyklus wird als Ist-Wert gemeldet.
  // Dadurch sieht der JointTrajectoryController immer Null-Fehler und
  // bleibt stabil, ohne echte Hardware zu benötigen.
  for (const auto& jname : {"joint_2", "joint_3", "joint_5", "joint_6", "joint_7"}) {
    if (joint_index_.count(jname)) {
      const size_t i = joint_index_.at(jname);
      hw_positions_[i]  = hw_cmd_positions_[i];
      hw_velocities_[i] = hw_cmd_velocities_[i];
      hw_efforts_[i]    = 0.0;
    }
  }

  // --- joint_8: Mimic von joint_7 (Multiplikator -1, rein passiv) ---
  if (joint_index_.count("joint_7") && joint_index_.count("joint_8")) {
    const size_t i7 = joint_index_.at("joint_7");
    const size_t i8 = joint_index_.at("joint_8");
    hw_positions_[i8]  = -hw_positions_[i7];
    hw_velocities_[i8] = -hw_velocities_[i7];
    hw_efforts_[i8]    = 0.0;
  }

  return hardware_interface::return_type::OK;
}

// ==============================================================================
// write(): Soll-Werte an Motoren senden
//   - joint_1, joint_4: CAN-Frame via Treiber
//   - Alle anderen:     kein Motor → ignoriert
// ==============================================================================
hardware_interface::return_type R0192SystemHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!can_available_) {
    return hardware_interface::return_type::OK;
  }

  if (joint_index_.count("joint_1")) {
    const size_t i = joint_index_.at("joint_1");
    axis1_->MIT_Control(hw_cmd_positions_[i], hw_cmd_velocities_[i], hw_cmd_kp_[i], hw_cmd_kd_[i], hw_cmd_efforts_[i]);
    //axis1_->Set_Input_Pos(hw_cmd_positions_[i], hw_cmd_velocities_[i], hw_cmd_efforts_[i]);
  }
  if (joint_index_.count("joint_4")) {
    const size_t i = joint_index_.at("joint_4");
    axis4_->MIT_Control(hw_cmd_positions_[i], hw_cmd_velocities_[i], hw_cmd_kp_[i], hw_cmd_kd_[i], hw_cmd_efforts_[i]);
  }
  return hardware_interface::return_type::OK;
}

// ==============================================================================
// canRxThread(): asynchroner CAN-Empfang im Hintergrund
// ==============================================================================
void R0192SystemHardware::canRxThread()
{
  struct can_frame frame;
  while (rx_thread_running_) {
    if (can_comm_->readFrame(frame)) {
      if (frame.can_id & CAN_EFF_FLAG) {
        // Extended Frame (29-Bit) → RS05
        uint32_t ext_id = frame.can_id & CAN_EFF_MASK;
        if (((ext_id >> 8) & 0xFF) == 0x04) {
          axis4_->processFeedbackFrame(frame);
        }
      } else {
        // Standard Frame (11-Bit) → GDS68: can_id = (node_id << 5) | cmd
        uint16_t std_id = frame.can_id & CAN_SFF_MASK;
        if ((std_id >> 5) == 0x01) {
          axis1_->processFeedbackFrame(frame);
        }
      }
    }
  }
}

}  // namespace r0192_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(r0192_hardware::R0192SystemHardware, hardware_interface::SystemInterface)
