#include "r0192_hardware/r0192_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace r0192_hardware
{

hardware_interface::CallbackReturn R0192SystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Speicher für die Arrays reservieren (basierend auf der Anzahl der Joints im URDF)
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_efforts_.resize(info_.joints.size(), 0.0);

  hw_cmd_positions_.resize(info_.joints.size(), 0.0);
  hw_cmd_velocities_.resize(info_.joints.size(), 0.0);
  hw_cmd_efforts_.resize(info_.joints.size(), 0.0);
  hw_cmd_kp_.resize(info_.joints.size(), 0.0);
  hw_cmd_kd_.resize(info_.joints.size(), 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn R0192SystemHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // CAN Bus öffnen und Treiber instanziieren
  can_comm_ = std::make_shared<CanCommunication>("can0");
  can_comm_->init();

  // Logger für Treiber erstellen
  auto logger = rclcpp::get_logger("R0192Hardware");
  axis1_ = std::make_shared<GDS68Driver>(0x01, can_comm_, logger);
  axis4_ = std::make_shared<RS05Driver>(0x04, can_comm_, logger);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn R0192SystemHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 1. RX Thread starten (für asynchrones CAN-Lesen)
  rx_thread_running_ = true;
  rx_thread_ = std::thread(&R0192SystemHardware::canRxThread, this);

  // 2. Motoren aktivieren (Hier rufen wir einmalig die Start-Befehle auf!)
  axis1_->Set_Axis_State(8); // GDS68 Closed Loop
  axis4_->Motor_Enabled_To_Run(); // RS05 Enable

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn R0192SystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Motoren ausschalten
  axis1_->Estop();
  axis4_->Motor_Stop_Running();

  // RX Thread beenden
  rx_thread_running_ = false;
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// --- ROS 2 Schnittstellen Registrierung ---
std::vector<hardware_interface::StateInterface> R0192SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> R0192SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_cmd_positions_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_cmd_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_cmd_efforts_[i]));
    // Eigene Interfaces für Impedanzregelung registrieren
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "kp", &hw_cmd_kp_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "kd", &hw_cmd_kd_[i]));
  }
  return command_interfaces;
}

// ==============================================================================
// === DIE ECHTZEIT-SCHLEIFE (vom ControllerManager strikt z.B. bei 500Hz aufgerufen)
// ==============================================================================

hardware_interface::return_type R0192SystemHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Lese die aktuellsten Werte aus den Treibern.
  // Dank der Mutexe in den Gettern ist das thread-safe!
  
  // Index 0: GDS68 (Achse 1)
  hw_positions_[0] = axis1_->get_current_position();
  hw_velocities_[0] = axis1_->get_current_velocity();
  hw_efforts_[0] = axis1_->get_current_torque();

  // Index 1: RS05 (Achse 4)
  hw_positions_[1] = axis4_->get_current_position();
  hw_velocities_[1] = axis4_->get_current_velocity();
  hw_efforts_[1] = axis4_->get_current_torque();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type R0192SystemHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Sende die neuen Soll-Werte aus ROS 2 über den CAN-Bus.
  // Hier werden die Arrays von einem Controller (z.B. JointTrajectoryController) befüllt.

  // Achse 1 (GDS68)
  axis1_->MIT_Control(
    hw_cmd_positions_[0], 
    hw_cmd_velocities_[0], 
    hw_cmd_kp_[0], 
    hw_cmd_kd_[0], 
    hw_cmd_efforts_[0]
  );

  // Achse 4 (RS05)
  axis4_->MIT_Control(
    hw_cmd_positions_[1], 
    hw_cmd_velocities_[1], 
    hw_cmd_kp_[1], 
    hw_cmd_kd_[1], 
    hw_cmd_efforts_[1]
  );

  return hardware_interface::return_type::OK;
}

// ==============================================================================
// === DER HINTERGRUND-THREAD FÜR CAN RX ===
// ==============================================================================
void R0192SystemHardware::canRxThread()
{
  struct can_frame frame;
  while (rx_thread_running_) {
    // Blockierendes (oder halblockierendes) Warten auf neue CAN-Nachrichten
    if (can_comm_->receiveFrame(&frame)) {
      if (frame.can_id & CAN_EFF_FLAG) {
         // Extended ID -> Wahrscheinlich RS05
         uint32_t ext_id = frame.can_id & CAN_EFF_MASK;
         if (((ext_id >> 8) & 0xFF) == 0x04) {
             axis4_->processFeedbackFrame(frame);
         }
      } else {
         // Standard ID -> Wahrscheinlich GDS68
         uint16_t std_id = frame.can_id & CAN_SFF_MASK;
         if ((std_id >> 5) == 0x01) {
             axis1_->processFeedbackFrame(frame);
         }
      }
    }
  }
}

}  // namespace r0192_hardware

// Plugin-Export für ROS 2
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  r0192_hardware::R0192SystemHardware, hardware_interface::SystemInterface)