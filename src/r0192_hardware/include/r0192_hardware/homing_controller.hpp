#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"

#include "r0192_canbus/CanCommunication.hpp"
#include "r0192_canbus/GDS68Driver.hpp"

namespace r0192_hardware
{

// ============================================================================
// HomingController
//
// Encapsulates the Arduino-based homing sequence for axis 1, separated from the
// ros2_control hardware interface to keep that file focused on the real-time
// read()/write() cycle.
//
// Owns its own ROS node ("r0192_homing"), the /homing service, and a dedicated
// executor thread so the blocking sequence never stalls the 100 Hz control loop.
//
// The hardware interface:
//   - constructs one HomingController once axis 1 is confirmed present,
//   - forwards Arduino CAN frames into notifyArduinoFrame() from its RX thread,
//   - queries isActive() in write() to suppress its own axis-1 MIT_Control
//     commands while a sweep is running.
// ============================================================================
class HomingController
{
public:
  // --- CAN protocol (Pi <-> Arduino), axis 1 ---
  // A single axis-specific ID is used for BOTH directions; the meaning lives in
  // data[0]. Axes 2-6 reuse this controller with a different AXIS_CAN_ID
  // (0x101 ... 0x105).
  static constexpr uint32_t AXIS_CAN_ID  = 0x100;  // axis 1 homing node
  static constexpr uint8_t  CMD_ARM      = 0x01;   // Pi -> Arduino: arm Hall sensor
  static constexpr uint8_t  RSP_DETECTED = 0xFF;   // Arduino -> Pi: magnet detected
  static constexpr uint8_t  RSP_ERROR    = 0xEE;   // Arduino -> Pi: timeout / fault

  // on_zeroed is invoked (from the homing thread) right after the encoder is
  // re-zeroed, so the hardware interface can sync its hw_positions_ /
  // hw_cmd_positions_ for joint_1 back to 0.
  HomingController(std::shared_ptr<GDS68Driver> axis1,
                   std::shared_ptr<CanCommunication> can_comm,
                   std::function<void()> on_zeroed);
  ~HomingController();

  // Create the node + /homing service and spin it on a background thread.
  void start();
  // Cancel the executor, join the thread and release the node/service.
  void stop();

  // Called from the hardware interface RX thread for every frame seen on
  // AXIS_CAN_ID. Records RSP_DETECTED / RSP_ERROR; ignores everything else
  // (notably CMD_ARM, in case own TX frames are looped back).
  void notifyArduinoFrame(uint8_t code);

  // True while a homing sweep is in progress → write() must skip axis 1.
  bool isActive() const { return active_.load(); }

private:
  // Service callback: runs the full edge-approach sequence and fills resp.
  void runHomingSequence(std::shared_ptr<std_srvs::srv::Trigger::Response> resp);

  // Arm the Arduino and watch briefly WITHOUT moving: true if the magnet is
  // already detected at the current position. Used to clear the magnet before
  // Pass 1 (otherwise the first edge detection would be mid-magnet/undefined).
  bool isOnMagnet();

  // Arm the Arduino and sweep axis 1 in `direction` (+1/-1) until RSP_DETECTED.
  // Drives a position setpoint ramped at homing_vel_ (rad/s) with search_kp_ so
  // the motor actually tracks the motion at a slow, controlled speed.
  // Returns the edge position, or NaN on timeout/RSP_ERROR (fail_reason_ holds
  // a human-readable cause).
  float findHomingEdge(float direction);

  // Move to `target` by ramping the position setpoint at move_vel_ (rad/s) so
  // the axis travels smoothly instead of snapping. Bounded by a 15 s deadline.
  void driveToPosition(float target);

  // Activate/deactivate managed_controller_ (the arm JointTrajectoryController)
  // via /controller_manager/switch_controller. Deactivating during homing stops
  // it fighting the sequence; reactivating afterwards makes it re-read the new
  // (zeroed) state instead of driving back to its stale pre-homing setpoint.
  // Returns true on success; logs a warning and returns false if unavailable.
  bool setArmControllerActive(bool active);

  std::shared_ptr<GDS68Driver>      axis1_;
  std::shared_ptr<CanCommunication> can_comm_;
  std::function<void()>             on_zeroed_;

  std::shared_ptr<rclcpp::Node>                        node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread                                          executor_thread_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr   service_;

  // Separate node for the switch_controller client so it can be spun on a
  // throwaway executor from inside the (blocking) /homing callback without
  // clashing with node_'s executor.
  std::shared_ptr<rclcpp::Node>                                            client_node_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_;

  std::atomic<bool>    active_{false};      // homing sweep in progress
  std::atomic<uint8_t> arduino_response_{0};// last Arduino code (0 = none yet)
  std::string          fail_reason_;        // set by findHomingEdge on failure

  // Tuning — overridable at runtime: ros2 param set /r0192_homing <name> <val>
  float  homing_vel_{0.15f};      // sweep speed (rad/s) of the ramped setpoint
  float  search_kp_{20.0f};       // position gain (KP) while sweeping for the edge
  float  homing_kd_{1.0f};        // velocity gain (KD) while sweeping
  float  move_vel_{0.5f};         // speed (rad/s) for overshoot & center moves
  float  hold_kp_{50.0f};         // position-hold KP for moves / after detection
  float  hold_kd_{1.0f};          // position-hold KD for moves / after detection
  float  overshoot_angle_{0.436f};// move past edge A before pass 2 (rad ~25 deg)
  float  zero_offset_{0.0f};      // magnet-midpoint -> desired zero (rad)
  float  search_dir_{-1.0f};      // Pass 1 sweep direction (+1/-1) toward the magnet
  float  max_start_angle_{3.1416f};// abort if |raw pos| exceeds this at start (rad, ~180 deg)
  double homing_timeout_{60.0};   // max seconds per approach direction (Pi side)
  std::string managed_controller_{"arm_controller"};  // paused/resumed around homing
};

}  // namespace r0192_hardware
