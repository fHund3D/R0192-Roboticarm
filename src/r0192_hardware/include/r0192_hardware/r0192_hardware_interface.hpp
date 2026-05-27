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
#include "std_srvs/srv/trigger.hpp"

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

  // --- Homing service (axis 1 only) ---
  // Lives in its own node + executor thread so the service handler can block
  // without interfering with the ros2_control real-time loop.
  // While homing_active_ is true, write() skips axis-1 MIT_Control commands.
  std::shared_ptr<rclcpp::Node>                               homing_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr         homing_executor_;
  std::thread                                                  homing_executor_thread_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr           homing_service_;
  std::atomic<bool> homing_active_{false};
  std::atomic<bool> arduino_ack_{false};

  // Homing tuning — changeable at runtime via `ros2 param set /r0192_homing ...`
  float  homing_vel_{0.15f};   // joint sweep speed (rad/s), KP=0 velocity mode
  float  homing_kd_{2.0f};     // velocity gain in MIT_Control during sweep
  float  hold_kp_{30.0f};      // position-hold KP after edge detection
  float  hold_kd_{1.0f};       // position-hold KD after edge detection
  float  zero_offset_{0.0f};   // offset from magnet midpoint to desired zero (rad)
  double homing_timeout_{60.0}; // max seconds per sweep direction

  // CAN IDs for the Arduino homing protocol
  static constexpr uint32_t HOMING_ARM_CAN_ID = 0x100; // Pi → Arduino: arm sensor
  static constexpr uint32_t HOMING_ACK_CAN_ID = 0x000; // Arduino → Pi: magnet detected
  static constexpr uint8_t  HOMING_ACK_VAL    = 0xFF;

  void runHomingSequence(std::shared_ptr<std_srvs::srv::Trigger::Response> resp);
  float findHomingEdge(float direction);   // returns edge position or NaN on timeout
  void  driveAxis1ToPosition(float target);
};

}  // namespace r0192_hardware