#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <thread>
#include <atomic>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

// Treiber
#include "r0192_canbus/CanCommunication.hpp"
#include "r0192_canbus/GDS68Driver.hpp"
#include "r0192_canbus/RS05Driver.hpp"

namespace r0192_hardware
{

class R0192SystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(R0192SystemHardware)

  // Lebenszyklus-Methoden (werden vom ControllerManager aufgerufen)
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // Schnittstellen-Registrierung
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Die harte Echtzeit-Schleife
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // --- Treiber & CAN ---
  std::shared_ptr<CanCommunication> can_comm_;
  std::shared_ptr<GDS68Driver> axis1_;
  std::shared_ptr<RS05Driver> axis4_;
  bool can_available_ = false;   // false if can0 was not up at configure time
  bool axis1_present_ = false;   // false if axis 1 did not respond to probe
  bool axis4_present_ = false;   // false if axis 4 did not respond to probe


  // --- RX Thread für asynchronen Empfang ---
  std::thread rx_thread_;
  std::atomic<bool> rx_thread_running_;
  void canRxThread();

  // --- Speicher für ros2_control ---
  // Vektoren für die Ist-Werte (werden in read() gefüllt)
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;

  // Vektoren für die Soll-Werte (werden in write() ausgelesen und an Motoren gesendet)
  std::vector<double> hw_cmd_positions_;
  std::vector<double> hw_cmd_velocities_;
  std::vector<double> hw_cmd_efforts_; // Torque / Effort
  std::vector<double> hw_cmd_kp_;
  std::vector<double> hw_cmd_kd_;

  // Joint name → vector index, built once in on_init()
  std::unordered_map<std::string, size_t> joint_index_;

  // Diagnostic: log commanded vs actual position every ~5 s at 100 Hz loop rate.
  int diag_counter_ = 0;
  static constexpr int diag_interval_ = 500;
};

}  // namespace r0192_hardware