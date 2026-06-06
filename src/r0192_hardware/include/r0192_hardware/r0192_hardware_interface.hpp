#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <thread>
#include <atomic>
#include <limits>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

// Treiber
#include "r0192_canbus/CanCommunication.hpp"
#include "r0192_canbus/GDS68Driver.hpp"
#include "r0192_canbus/RS05Driver.hpp"

// Homing (axis 1) — lives in its own translation unit
#include "r0192_hardware/homing_controller.hpp"

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

  // --- Sicherheits-Check beim Start ---
  // Prüft, ob jede physisch vorhandene Achse (axis1/axis4) laut RAW-Encoder
  // innerhalb ihrer URDF-Positionslimits (info_.limits) steht. Liefert false
  // und füllt `reason`, wenn eine Achse außerhalb liegt. Virtuelle Achsen haben
  // keinen Encoder und werden übersprungen.
  bool encodersWithinLimits(std::string & reason);

  // Motoren stoppen und CAN-RX-Thread beenden — gemeinsam genutzt von
  // on_deactivate() und dem Fehlerpfad bei abgebrochener Aktivierung.
  void stopMotorsAndRx();

  // --- Motor enable/disable (/robot_enable, std_srvs/SetBool) ---
  // Operator-Schnittstelle (RViz-Panel / CLI), um das Motor-Drehmoment zur
  // Laufzeit zu kappen bzw. wiederherzustellen, ohne den Lifecycle zu wechseln.
  // write() sendet MIT_Control nur, solange motors_enabled_ true ist.
  // Eigener Node + Executor-Thread, damit der Service-Callback die 100-Hz-
  // read()/write()-Schleife nicht stört (CAN-Sends sind via send_mutex_ safe).
  void setMotorsEnabled(bool enable);
  std::atomic<bool> motors_enabled_{true};

  // --- Hardware emergency stop + reset (/robot_estop, /robot_reset) ---
  // /robot_estop cuts torque at the DRIVER level (GDS68 Estop() 0x002 latches the
  // axis into an error state; RS05 stop), not just by gating write(). The drivers
  // then stay "trapped" until /robot_reset clears the faults (GDS68 Clear_Errors()
  // 0x018; RS05 stop with clear-faults bit). estop_latched_ records that a reset
  // is required before the motors can be re-enabled. Both share enable_node_.
  void triggerEstop();
  void resetDrivers();
  std::atomic<bool> estop_latched_{false};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr  estop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr  reset_service_;

  std::shared_ptr<rclcpp::Node>                        enable_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr enable_executor_;
  std::thread                                          enable_executor_thread_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr   enable_service_;

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

  // RS05 (axis 4) feedback-while-disabled poll throttle. The RS05 has no
  // ODrive-style idle: when stopped (Reset mode) it streams neither MIT
  // responses nor Type-24 active reports. While motors are disabled, read()
  // polls mechPos/mechVel via Type-17 reads (read-only, no torque) at
  // ~100/rs05_poll_decim_ Hz so joint_4 still tracks when back-driven.
  uint32_t rs05_poll_tick_ = 0;
  static constexpr uint32_t rs05_poll_decim_ = 5;  // 100 Hz / 5 = 20 Hz

  // --- Homing (axis 1 only) ---
  // Created in on_activate() once axis 1 is confirmed present; owns its own
  // node + /homing service + executor thread (see homing_controller.hpp).
  // canRxThread() forwards AXIS_CAN_ID frames into it; write() skips axis 1
  // while homing_->isActive().
  std::shared_ptr<HomingController> homing_;
};

}  // namespace r0192_hardware