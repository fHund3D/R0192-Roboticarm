#include "r0192_hardware/r0192_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

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

    RCLCPP_INFO(logger, "Probing physical axes (200 ms timeout each)...");
    axis1_present_ = axis1_->probePresent(200);
    axis4_present_ = axis4_->probePresent(200);

    if (!axis1_present_ && !axis4_present_) {
      RCLCPP_WARN(logger, "No physical axes detected — running in full virtual mode");
    } else {
      if (!axis1_present_) RCLCPP_WARN(logger, "Axis 1 (GDS68) not detected — virtual passthrough");
      if (!axis4_present_) RCLCPP_WARN(logger, "Axis 4 (RS05) not detected — virtual passthrough");
    }
    RCLCPP_INFO(logger, "R0192 hardware configured (CAN 'can0' open, axis1=%s axis4=%s)",
      axis1_present_ ? "real" : "virtual", axis4_present_ ? "real" : "virtual");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn R0192SystemHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = rclcpp::get_logger("R0192Hardware");
  motors_enabled_ = true;  // reset in case a previous cycle left motors disabled

  if (can_available_) {
    rx_thread_running_ = true;
    rx_thread_ = std::thread(&R0192SystemHardware::canRxThread, this);

    // --- Enable motors and request encoder feedback ---
    // (Done before the safety check below so the raw encoder readings are valid;
    //  actual zeroing happens only after the limit check passes.)
    if (axis1_present_) {
      axis1_->Set_Axis_State(8);
      // Position Control (3) + Passthrough (1) to prevent drift.
      axis1_->Set_Controller_Mode(3, 1);
      axis1_->Get_Encoder_Estimates();
    }
    if (axis4_present_) {
      axis4_->Motor_Enabled_To_Run();
      // Enable active reporting (Type 24, F_CMD=01 → 10 ms = 100 Hz) so the RS05
      // streams its Type-2 feedback frame continuously, independent of
      // MIT_Control / motor enable state. Without this the RS05 only replies
      // with feedback while we actively command it, so a disabled motor would
      // report nothing — same goal as the GDS68 0x009 periodic frame. Set purely
      // over CAN (no tool needed); not persisted to flash, so it is re-sent on
      // every activation. The Type-2 frame is parsed identically whether it
      // arrives as a MIT response or an active report, so no read-path change is
      // needed.
      axis4_->Actively_Reports_Frame(1.0f);
    }
    // Let the first feedback frame stream back before reading the encoders.
    if (axis1_present_ || axis4_present_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // --- Safety check: refuse to activate if any physical axis is parked
    //     outside its URDF position limits (checked on the RAW encoder reading,
    //     i.e. before any zeroing). Returning ERROR keeps the component out of
    //     the active state — the motors are stopped and the RX thread joined. ---
    std::string limit_violation;
    if (!encodersWithinLimits(limit_violation)) {
      RCLCPP_ERROR(logger,
        "Activation aborted — %s. Move the axis within its joint limits and retry.",
        limit_violation.c_str());
      stopMotorsAndRx();
      return hardware_interface::CallbackReturn::ERROR;
    }

    // --- Zeroing (axis 1, GDS68) ---
    if (axis1_present_ && joint_index_.count("joint_1")) {
      const size_t i = joint_index_.at("joint_1");
      RCLCPP_INFO(logger, "Axis 1 raw encoder: %.3f rad — zeroing at current position",
                  axis1_->get_current_position());
      axis1_->Set_Linear_Count(0);
      RCLCPP_INFO(logger, "Axis 1 mechanical zero set");
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      axis1_->Get_Encoder_Estimates();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      double pos = axis1_->get_current_position();
      RCLCPP_INFO(logger, "Axis 1 after zero: %.3f rad", pos);
      hw_cmd_positions_[i] = pos;
    }

    // --- Zeroing (axis 4, RS05) ---
    if (axis4_present_ && joint_index_.count("joint_4")) {
      const size_t i = joint_index_.at("joint_4");
      RCLCPP_INFO(logger, "Axis 4 raw encoder: %.3f rad — zeroing at current position",
                  axis4_->get_current_position());
      axis4_->Set_Motor_Mechanical_Zero();
      RCLCPP_INFO(logger, "Axis 4 mechanical zero set");
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      double pos = axis4_->get_current_position();
      RCLCPP_INFO(logger, "Axis 4 after zero: %.3f rad", pos);
      hw_cmd_positions_[i] = pos;
    }

    if (!axis1_present_ && !axis4_present_) {
      RCLCPP_WARN(logger, "Activated in virtual mode — no CAN frames will be sent or received");
    }

    // --- Homing (axis 1) ---
    if (axis1_present_) {
      // on_zeroed callback: after axis 1 is re-zeroed, snap the whole arm to the
      // home pose (all joints 0) so RViz/MoveIt show a clean zero and no phantom
      // offsets appear when the controller resumes. State+command are forced to
      // 0 for the homed axis and all virtual/passthrough joints. A REAL,
      // non-homed axis (only joint_4 can be one) is skipped: its state comes
      // from CAN feedback and forcing a 0 command would physically drive it.
      auto on_zeroed = [this]() {
        for (size_t i = 0; i < info_.joints.size(); ++i) {
          if (info_.joints[i].name == "joint_4" && axis4_present_) continue;
          hw_positions_[i]      = 0.0;
          hw_velocities_[i]     = 0.0;
          hw_cmd_positions_[i]  = 0.0;
          hw_cmd_velocities_[i] = 0.0;  // also clears stale velocity feed-forward
        }
      };
      homing_ = std::make_shared<HomingController>(axis1_, can_comm_, on_zeroed);
      homing_->start();
    }

    // --- Start de-energized ---
    // Activation only powered the axes long enough for the safety check + zeroing
    // above. Leave the motors idle/torque-free now; the operator turns them on
    // deliberately via /robot_enable (RViz operator panel). Until then write()
    // sends no MIT_Control (gated on motors_enabled_).
    if (axis1_present_ || axis4_present_) {
      motors_enabled_ = false;
      if (axis1_present_) axis1_->Set_Axis_State(1);
      if (axis4_present_) axis4_->Motor_Stop_Running();
      RCLCPP_INFO(logger,
        "Motors start DISABLED — enable via /robot_enable (RViz operator panel)");
    }
  } else {
    RCLCPP_WARN(logger, "Activated in virtual mode — no CAN frames will be sent or received");
  }

  // --- /robot_enable service (motor torque on/off for the operator panel) ---
  enable_node_ = std::make_shared<rclcpp::Node>("r0192_robot_enable");
  enable_service_ = enable_node_->create_service<std_srvs::srv::SetBool>(
    "/robot_enable",
    [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
           std::shared_ptr<std_srvs::srv::SetBool::Response> resp) {
      setMotorsEnabled(req->data);
      resp->success = true;
      resp->message = req->data ? "Motoren aktiviert" : "Motoren deaktiviert";
    });
  enable_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  enable_executor_->add_node(enable_node_);
  enable_executor_thread_ = std::thread([this]() { enable_executor_->spin(); });
  RCLCPP_INFO(logger, "Motor enable service ready at /robot_enable");

  RCLCPP_INFO(logger, "R0192 hardware activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn R0192SystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto logger = rclcpp::get_logger("R0192Hardware");

  // Motor-enable service teardown.
  if (enable_executor_) {
    enable_executor_->cancel();
    if (enable_executor_thread_.joinable()) enable_executor_thread_.join();
    enable_executor_.reset();
  }
  enable_service_.reset();
  enable_node_.reset();

  // Homing teardown (before stopping motors so the isActive() gate in write()
  // still behaves while the service thread winds down).
  if (homing_) {
    homing_->stop();
    homing_.reset();
  }

  if (can_available_) {
    stopMotorsAndRx();
  }

  RCLCPP_INFO(logger, "R0192 hardware deactivated");
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
  // RS05 feedback while disabled: the RS05 has no ODrive-style idle, so once it
  // is stopped (Reset mode) it streams neither MIT responses nor Type-24 active
  // reports. Poll mechPos/mechVel via Type-17 reads (read-only — does NOT
  // energize the motor) so joint_4 keeps tracking when back-driven by hand.
  // Throttled to ~20 Hz; the async replies update current_pos_/vel_ in the
  // driver (processFeedbackFrame case 0x11). When enabled, active reporting +
  // MIT responses cover feedback, so polling is skipped.
  if (!motors_enabled_ && axis4_present_ && (rs05_poll_tick_++ % rs05_poll_decim_ == 0)) {
    axis4_->Single_Parameter_Read(RS05Driver::PARAM_MECH_POS);
    axis4_->Single_Parameter_Read(RS05Driver::PARAM_MECH_VEL);
  }

  // joint_1: real CAN feedback if probed present, otherwise passthrough
  if (joint_index_.count("joint_1")) {
    const size_t i = joint_index_.at("joint_1");
    if (axis1_present_) {
      hw_positions_[i] = axis1_->get_current_position();
      hw_velocities_[i] = axis1_->get_current_velocity();
      hw_efforts_[i]    = axis1_->get_current_torque();
    } else {
      hw_positions_[i]  = hw_cmd_positions_[i];
      hw_velocities_[i] = hw_cmd_velocities_[i];
      hw_efforts_[i]    = 0.0;
    }
  }

  // joint_4: real CAN feedback if probed present, otherwise passthrough
  if (joint_index_.count("joint_4")) {
    const size_t i = joint_index_.at("joint_4");
    if (axis4_present_) {
      hw_positions_[i] = axis4_->get_current_position();
      hw_velocities_[i] = axis4_->get_current_velocity();
      hw_efforts_[i]    = axis4_->get_current_torque();
    } else {
      hw_positions_[i]  = hw_cmd_positions_[i];
      hw_velocities_[i] = hw_cmd_velocities_[i];
      hw_efforts_[i]    = 0.0;
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
  if (!motors_enabled_) {
    // Torque cut via /robot_enable — send no MIT_Control until re-enabled.
    return hardware_interface::return_type::OK;
  }
  if (axis1_present_ && joint_index_.count("joint_1") && !(homing_ && homing_->isActive())) {
    const size_t i = joint_index_.at("joint_1");
    axis1_->MIT_Control(hw_cmd_positions_[i], hw_cmd_velocities_[i], hw_cmd_kp_[i], hw_cmd_kd_[i], hw_cmd_efforts_[i]);
  }
  if (axis4_present_ && joint_index_.count("joint_4")) {
    const size_t i = joint_index_.at("joint_4");
    axis4_->MIT_Control(hw_cmd_positions_[i], hw_cmd_velocities_[i], hw_cmd_kp_[i], hw_cmd_kd_[i], hw_cmd_efforts_[i]);
  }
  return hardware_interface::return_type::OK;
}

// ==============================================================================
// setMotorsEnabled(): /robot_enable callback — Motor-Drehmoment an/aus
//   true  → Achsen scharf schalten (Closed-Loop) und Soll = aktuelle Ist-Pos
//           setzen (kein Snap im selben Zyklus); write() sendet wieder.
//   false → write() gibt nichts mehr aus und die Achsen werden in Idle/Stop
//           geschaltet (Drehmoment weg).
//   ACHTUNG: Bei aktivem JointTrajectoryController überschreibt dieser das
//   Soll im nächsten Zyklus wieder. Driftet eine Achse im deaktivierten
//   Zustand (Schwerkraft), kann es beim Re-Enable einen Ruck geben. Achse 1
//   (vertikale Drehachse) ist davon praktisch nicht betroffen.
// ==============================================================================
void R0192SystemHardware::setMotorsEnabled(bool enable)
{
  auto logger = rclcpp::get_logger("R0192Hardware");
  if (!can_available_) {
    RCLCPP_WARN(logger, "/robot_enable ignored — running in virtual mode (no CAN)");
    return;
  }

  if (enable) {
    if (axis1_present_) {
      axis1_->Set_Axis_State(8);
      axis1_->Set_Controller_Mode(3, 1);
      if (joint_index_.count("joint_1"))
        hw_cmd_positions_[joint_index_.at("joint_1")] = axis1_->get_current_position();
    }
    if (axis4_present_) {
      axis4_->Motor_Enabled_To_Run();
      // Re-arm active reporting: Reset mode (from the previous disable) stops
      // Type-2 streaming, and re-entering Run mode may not resume it, so re-send
      // the Type-24 enable on every enable (idempotent, cheap insurance).
      axis4_->Actively_Reports_Frame(1.0f);
      if (joint_index_.count("joint_4"))
        hw_cmd_positions_[joint_index_.at("joint_4")] = axis4_->get_current_position();
    }
    motors_enabled_ = true;
    RCLCPP_INFO(logger, "Motors ENABLED via /robot_enable");
  } else {
    motors_enabled_ = false;  // stop write() before cutting torque
    if (axis1_present_) axis1_->Set_Axis_State(1);
    if (axis4_present_) axis4_->Motor_Stop_Running();
    RCLCPP_WARN(logger, "Motors DISABLED via /robot_enable");
  }
}

// ==============================================================================
// encodersWithinLimits(): Start-Sicherheits-Check
//   Jede physisch vorhandene Achse muss laut RAW-Encoder innerhalb ihrer
//   URDF-Positionslimits stehen. Virtuelle Achsen haben keinen Encoder und
//   werden übersprungen. Bei Verletzung → reason gesetzt, false zurück.
// ==============================================================================
bool R0192SystemHardware::encodersWithinLimits(std::string & reason)
{
  struct PhysAxis { const char * joint; double pos; };
  std::vector<PhysAxis> axes;
  if (axis1_present_ && joint_index_.count("joint_1")) {
    axes.push_back({"joint_1", axis1_->get_current_position()});
  }
  if (axis4_present_ && joint_index_.count("joint_4")) {
    axes.push_back({"joint_4", axis4_->get_current_position()});
  }

  for (const auto & a : axes) {
    const auto it = info_.limits.find(a.joint);
    if (it == info_.limits.end() || !it->second.has_position_limits) {
      continue;  // no URDF position limit declared → nothing to enforce
    }
    const double lo = it->second.min_position;
    const double hi = it->second.max_position;
    if (a.pos < lo || a.pos > hi) {
      char buf[192];
      std::snprintf(buf, sizeof(buf),
        "%s encoder at %.3f rad is outside its URDF limits [%.3f, %.3f] rad",
        a.joint, a.pos, lo, hi);
      reason = buf;
      return false;
    }
  }
  return true;
}

// ==============================================================================
// stopMotorsAndRx(): Motoren stoppen + CAN-RX-Thread beenden
//   Gemeinsam genutzt von on_deactivate() und dem Fehlerpfad in on_activate().
// ==============================================================================
void R0192SystemHardware::stopMotorsAndRx()
{
  if (axis1_present_) axis1_->Set_Axis_State(1);
  if (axis4_present_) {
    axis4_->Actively_Reports_Frame(0.0f);  // stop Type-2 streaming (bus cleanup)
    axis4_->Motor_Stop_Running();
  }

  rx_thread_running_ = false;
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
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
        // Standard Frame (11-Bit)
        uint16_t std_id = frame.can_id & CAN_SFF_MASK;

        // Arduino homing response: shares AXIS_CAN_ID with the arm command,
        // distinguished by data[0] (RSP_DETECTED / RSP_ERROR).
        if (homing_ && std_id == HomingController::AXIS_CAN_ID && frame.can_dlc >= 1) {
          homing_->notifyArduinoFrame(frame.data[0]);
        }

        // GDS68 feedback: can_id = (node_id << 5) | cmd  → node_id == 0x01
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
