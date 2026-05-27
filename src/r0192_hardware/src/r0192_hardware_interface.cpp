#include "r0192_hardware/r0192_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cmath>

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

  if (can_available_) {
    rx_thread_running_ = true;
    rx_thread_ = std::thread(&R0192SystemHardware::canRxThread, this);

    // GDS68 (axis 1)
    if (axis1_present_) {
      axis1_->Set_Axis_State(8);
      // Position Control (3) + Passthrough (1) to prevent drift.
      axis1_->Set_Controller_Mode(3, 1);

      if (joint_index_.count("joint_1")) {
        const size_t i = joint_index_.at("joint_1");
        axis1_->Get_Encoder_Estimates();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
    }

    // RS05 (axis 4)
    if (axis4_present_) {
      axis4_->Motor_Enabled_To_Run();

      if (joint_index_.count("joint_4")) {
        const size_t i = joint_index_.at("joint_4");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        RCLCPP_INFO(logger, "Axis 4 raw encoder: %.3f rad — zeroing at current position",
                    axis4_->get_current_position());
        axis4_->Set_Motor_Mechanical_Zero();
        RCLCPP_INFO(logger, "Axis 4 mechanical zero set");
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        double pos = axis4_->get_current_position();
        RCLCPP_INFO(logger, "Axis 4 after zero: %.3f rad", pos);
        hw_cmd_positions_[i] = pos;
      }
    }

    if (!axis1_present_ && !axis4_present_) {
      RCLCPP_WARN(logger, "Activated in virtual mode — no CAN frames will be sent or received");
    }

    // --- Homing service (axis 1) ---
    if (axis1_present_) {
      homing_node_ = std::make_shared<rclcpp::Node>("r0192_homing");

      // Allow overriding tuning params at runtime:  ros2 param set /r0192_homing homing_vel 0.1
      homing_vel_     = static_cast<float>(homing_node_->declare_parameter("homing_vel",     0.15));
      homing_kd_      = static_cast<float>(homing_node_->declare_parameter("homing_kd",      2.0));
      hold_kp_        = static_cast<float>(homing_node_->declare_parameter("hold_kp",        30.0));
      hold_kd_        = static_cast<float>(homing_node_->declare_parameter("hold_kd",        1.0));
      zero_offset_    = static_cast<float>(homing_node_->declare_parameter("zero_offset",    0.0));
      homing_timeout_ = homing_node_->declare_parameter("homing_timeout", 60.0);

      homing_service_ = homing_node_->create_service<std_srvs::srv::Trigger>(
        "/homing",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
          runHomingSequence(resp);
        });

      homing_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      homing_executor_->add_node(homing_node_);
      homing_executor_thread_ = std::thread([this]() { homing_executor_->spin(); });

      RCLCPP_INFO(logger, "Homing service ready at /homing");
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

  // Homing service teardown (before stopping motors so homing_active_ check in write() still works)
  if (homing_executor_) {
    homing_executor_->cancel();
    if (homing_executor_thread_.joinable()) homing_executor_thread_.join();
    homing_executor_.reset();
    homing_service_.reset();
    homing_node_.reset();
  }

  if (can_available_) {
    if (axis1_present_) axis1_->Set_Axis_State(1);
    if (axis4_present_) axis4_->Motor_Stop_Running();

    rx_thread_running_ = false;
    if (rx_thread_.joinable()) {
      rx_thread_.join();
    }
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
  if (axis1_present_ && joint_index_.count("joint_1") && !homing_active_.load()) {
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

        // Arduino homing ACK: ID 0x000, data[0] == 0xFF → magnet detected
        if (std_id == HOMING_ACK_CAN_ID && frame.can_dlc >= 1 && frame.data[0] == HOMING_ACK_VAL) {
          RCLCPP_INFO(rclcpp::get_logger("R0192Hardware"), "Homing: Arduino ACK received — magnet detected");
          arduino_ack_.store(true);
        }

        // GDS68 feedback: can_id = (node_id << 5) | cmd  → node_id == 0x01
        if ((std_id >> 5) == 0x01) {
          axis1_->processFeedbackFrame(frame);
        }
      }
    }
  }
}

// ==============================================================================
// Homing sequence — runs in the homing executor thread (blocking service cb)
// ==============================================================================

// Arm the Arduino and drive axis 1 at homing_vel_ until the Hall sensor fires.
// Returns the joint position at detection, or NaN on timeout.
float R0192SystemHardware::findHomingEdge(float direction)
{
  using namespace std::chrono_literals;
  auto logger = rclcpp::get_logger("R0192Hardware");

  arduino_ack_.store(false);

  // Arm the Arduino homing sensor node for axis 1
  uint8_t arm_data[1] = {0x01};
  can_comm_->sendFrame(HOMING_ARM_CAN_ID, 1, arm_data);
  RCLCPP_INFO(logger, "Homing: Arduino armed — sweeping axis 1 in %+.0f direction", (double)direction);
  std::this_thread::sleep_for(100ms);

  auto deadline = std::chrono::steady_clock::now() +
                  std::chrono::duration<double>(homing_timeout_);

  float vel = direction * homing_vel_;
  while (!arduino_ack_.load()) {
    if (std::chrono::steady_clock::now() >= deadline) {
      RCLCPP_ERROR(logger, "Homing: timeout (%.0f s) — no Hall signal received", homing_timeout_);
      float pos = axis1_->get_current_position();
      axis1_->MIT_Control(pos, 0.0f, hold_kp_, hold_kd_, 0.0f);
      return std::numeric_limits<float>::quiet_NaN();
    }
    // Velocity-mode MIT: KP=0, vel_ref=vel, KD=velocity_gain
    float pos = axis1_->get_current_position();
    axis1_->MIT_Control(pos, vel, 0.0f, homing_kd_, 0.0f);    //Anpassen!!!!!!!!!!!!!!!
    std::this_thread::sleep_for(20ms);
  }

  float edge = axis1_->get_current_position();
  axis1_->MIT_Control(edge, 0.0f, hold_kp_, hold_kd_, 0.0f);
  RCLCPP_INFO(logger, "Homing: edge detected at %.4f rad", (double)edge);
  return edge;
}

// Drive axis 1 to target and wait until it arrives (tolerance 0.02 rad ≈ 1°).
void R0192SystemHardware::driveAxis1ToPosition(float target)
{
  using namespace std::chrono_literals;
  auto logger  = rclcpp::get_logger("R0192Hardware");
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);

  RCLCPP_INFO(logger, "Homing: driving to zero target %.4f rad", (double)target);
  while (std::chrono::steady_clock::now() < deadline) {
    float pos = axis1_->get_current_position();
    axis1_->MIT_Control(target, 0.0f, hold_kp_, hold_kd_, 0.0f);
    if (std::abs(pos - target) < 0.02f) break;
    std::this_thread::sleep_for(20ms);
  }
  RCLCPP_INFO(logger, "Homing: arrived at zero target");
}

// Full bisection homing sequence (called from the /homing service callback).
void R0192SystemHardware::runHomingSequence(
  std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
{
  using namespace std::chrono_literals;
  auto logger = rclcpp::get_logger("R0192Hardware");

  if (!axis1_present_) {
    resp->success = false;
    resp->message = "Axis 1 not present — homing aborted";
    RCLCPP_ERROR(logger, "%s", resp->message.c_str());
    return;
  }

  RCLCPP_INFO(logger, "===== HOMING SEQUENCE START =====");

  // Gate ros2_control write() for axis 1 while homing
  homing_active_.store(true);

  // Ensure motor is enabled in position-control mode
  axis1_->Set_Axis_State(8);
  axis1_->Set_Controller_Mode(3, 1);
  std::this_thread::sleep_for(300ms);

  // Pass 1: sweep in positive direction → P1
  float p1 = findHomingEdge(+1.0f);
  if (std::isnan(p1)) {
    homing_active_.store(false);
    resp->success = false;
    resp->message = "Homing failed: timeout on forward sweep";
    RCLCPP_ERROR(logger, "%s", resp->message.c_str());
    return;
  }
  std::this_thread::sleep_for(500ms);

  // Pass 2: sweep in negative direction → P2
  float p2 = findHomingEdge(-1.0f);
  if (std::isnan(p2)) {
    homing_active_.store(false);
    resp->success = false;
    resp->message = "Homing failed: timeout on reverse sweep";
    RCLCPP_ERROR(logger, "%s", resp->message.c_str());
    return;
  }
  std::this_thread::sleep_for(500ms);

  // Bisect: magnet midpoint + optional offset = desired zero
  float center      = (p1 + p2) / 2.0f;
  float zero_target = center + zero_offset_;
  RCLCPP_INFO(logger,
    "Homing: P1=%.4f  P2=%.4f  center=%.4f  offset=%.4f  → zero_target=%.4f",
    (double)p1, (double)p2, (double)center, (double)zero_offset_, (double)zero_target);

  driveAxis1ToPosition(zero_target);
  std::this_thread::sleep_for(300ms);

  // Zero the encoder at the current (physical zero) position
  axis1_->Set_Linear_Count(0);
  RCLCPP_INFO(logger, "Homing: encoder zeroed via Set_Linear_Count(0)");
  std::this_thread::sleep_for(100ms);

  // Sync the ros2_control state: report position 0 so joint_state_broadcaster
  // reflects the new zero immediately when write() resumes.
  if (joint_index_.count("joint_1")) {
    const size_t i = joint_index_.at("joint_1");
    hw_positions_[i]     = 0.0;
    hw_cmd_positions_[i] = 0.0;  // prevent arm_controller from driving back to pre-homing pos
  }

  homing_active_.store(false);  // re-enable write() for axis 1

  resp->success = true;
  resp->message = "Homing complete — axis 1 at zero";
  RCLCPP_INFO(logger, "===== HOMING COMPLETE =====");
}

}  // namespace r0192_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(r0192_hardware::R0192SystemHardware, hardware_interface::SystemInterface)
