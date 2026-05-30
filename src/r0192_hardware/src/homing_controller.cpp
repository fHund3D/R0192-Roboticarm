#include "r0192_hardware/homing_controller.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>

namespace r0192_hardware
{

using namespace std::chrono_literals;

HomingController::HomingController(std::shared_ptr<GDS68Driver> axis1,
                                   std::shared_ptr<CanCommunication> can_comm,
                                   std::function<void()> on_zeroed)
: axis1_(std::move(axis1)),
  can_comm_(std::move(can_comm)),
  on_zeroed_(std::move(on_zeroed))
{
}

HomingController::~HomingController()
{
  stop();
}

void HomingController::start()
{
  node_ = std::make_shared<rclcpp::Node>("r0192_homing");

  // Allow overriding tuning params at runtime, e.g.
  //   ros2 param set /r0192_homing homing_vel 0.1
  homing_vel_      = static_cast<float>(node_->declare_parameter("homing_vel",      0.3));
  search_kp_       = static_cast<float>(node_->declare_parameter("search_kp",       20.0));
  homing_kd_       = static_cast<float>(node_->declare_parameter("homing_kd",       1.0));
  move_vel_        = static_cast<float>(node_->declare_parameter("move_vel",        0.5));
  hold_kp_         = static_cast<float>(node_->declare_parameter("hold_kp",         50.0));
  hold_kd_         = static_cast<float>(node_->declare_parameter("hold_kd",         1.0));
  overshoot_angle_ = static_cast<float>(node_->declare_parameter("overshoot_angle", 0.436));  //größer anpassen!!!
  zero_offset_     = static_cast<float>(node_->declare_parameter("zero_offset",     0.0));
  search_dir_      = static_cast<float>(node_->declare_parameter("search_dir",      -1.0));
  max_start_angle_ = static_cast<float>(node_->declare_parameter("max_start_angle", M_PI));
  homing_timeout_  = node_->declare_parameter("homing_timeout", 60.0);
  search_dir_      = (search_dir_ >= 0.0f) ? 1.0f : -1.0f;  // normalize to ±1
  managed_controller_ = node_->declare_parameter("managed_controller", std::string("arm_controller"));

  service_ = node_->create_service<std_srvs::srv::Trigger>(
    "/homing",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
           std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
      runHomingSequence(resp);
    });

  // Client (on its own node) to pause/resume the arm controller around homing.
  client_node_   = std::make_shared<rclcpp::Node>("r0192_homing_client");
  switch_client_ = client_node_->create_client<controller_manager_msgs::srv::SwitchController>(
    "/controller_manager/switch_controller");

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  executor_thread_ = std::thread([this]() { executor_->spin(); });

  RCLCPP_INFO(node_->get_logger(), "Homing service ready at /homing");
}

void HomingController::stop()
{
  if (executor_) {
    executor_->cancel();
    if (executor_thread_.joinable()) executor_thread_.join();
    executor_.reset();
  }
  service_.reset();
  node_.reset();
  switch_client_.reset();
  client_node_.reset();
}

void HomingController::notifyArduinoFrame(uint8_t code)
{
  // Only record genuine responses; ignore CMD_ARM in case our own TX frame on
  // AXIS_CAN_ID is ever looped back to the RX socket.
  if (code == RSP_DETECTED || code == RSP_ERROR) {
    arduino_response_.store(code);
  }
}

// Arm the Arduino and hold position for a short window; return true if the
// magnet is detected immediately (i.e. the axis is already sitting on it).
bool HomingController::isOnMagnet()
{
  auto logger = node_->get_logger();

  arduino_response_.store(0);
  uint8_t arm_data[1] = {CMD_ARM};
  can_comm_->sendFrame(AXIS_CAN_ID, 1, arm_data);

  const float hold = axis1_->get_current_position();
  auto deadline = std::chrono::steady_clock::now() + 500ms;
  while (std::chrono::steady_clock::now() < deadline) {
    if (arduino_response_.load() == RSP_DETECTED) {
      RCLCPP_INFO(logger, "Homing: axis already on magnet at start");
      return true;
    }
    axis1_->MIT_Control(hold, 0.0f, hold_kp_, hold_kd_, 0.0f);  // hold still
    std::this_thread::sleep_for(20ms);
  }
  return false;  // not on magnet (Arduino stays armed; Pass 1 re-arms it)
}

// Arm the Arduino and sweep axis 1 until the Hall sensor fires (RSP_DETECTED).
// Returns the edge position, or NaN on RSP_ERROR / Pi-side timeout.
float HomingController::findHomingEdge(float direction)
{
  auto logger = node_->get_logger();

  arduino_response_.store(0);

  // Arm the Arduino homing node for this axis
  uint8_t arm_data[1] = {CMD_ARM};
  can_comm_->sendFrame(AXIS_CAN_ID, 1, arm_data);
  RCLCPP_INFO(logger, "Homing: Arduino armed — sweeping axis 1 in %+.0f direction", (double)direction);
  std::this_thread::sleep_for(100ms);

  auto deadline = std::chrono::steady_clock::now() +
                  std::chrono::duration<double>(homing_timeout_);

  // Ramp a position setpoint at homing_vel_ (with search_kp_ position authority
  // + velocity feed-forward) so the motor actually drives itself slowly toward
  // the sensor instead of relying on a weak velocity-only torque.
  constexpr float dt   = 0.02f;                 // 20 ms control loop
  const float     vel  = direction * homing_vel_;
  const float     step = vel * dt;              // setpoint increment per loop
  float setpoint = axis1_->get_current_position();

  while (true) {
    const uint8_t resp = arduino_response_.load();
    if (resp == RSP_DETECTED) break;
    if (resp == RSP_ERROR) {
      fail_reason_ = "Arduino reported RSP_ERROR (Hall timeout/fault)";
      RCLCPP_ERROR(logger, "Homing: %s", fail_reason_.c_str());
      float pos = axis1_->get_current_position();
      axis1_->MIT_Control(pos, 0.0f, hold_kp_, hold_kd_, 0.0f);  // hold in place
      return std::numeric_limits<float>::quiet_NaN();
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      fail_reason_ = "Pi-side timeout — no Arduino response received";
      RCLCPP_ERROR(logger, "Homing: timeout (%.0f s) — no Hall signal received", homing_timeout_);
      float pos = axis1_->get_current_position();
      axis1_->MIT_Control(pos, 0.0f, hold_kp_, hold_kd_, 0.0f);  // hold in place
      return std::numeric_limits<float>::quiet_NaN();
    }
    setpoint += step;
    axis1_->MIT_Control(setpoint, vel, search_kp_, homing_kd_, 0.0f);
    std::this_thread::sleep_for(20ms);
  }

  float edge = axis1_->get_current_position();
  axis1_->MIT_Control(edge, 0.0f, hold_kp_, hold_kd_, 0.0f);  // hold at edge
  RCLCPP_INFO(logger, "Homing: edge detected at %.4f rad", (double)edge);
  return edge;
}

void HomingController::driveToPosition(float target)
{
  auto logger   = node_->get_logger();
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);

  // Ramp the setpoint from the current position to `target` at move_vel_ so the
  // axis travels smoothly instead of snapping to the goal under stiff hold_kp_.
  constexpr float dt    = 0.02f;                // 20 ms control loop
  float       setpoint  = axis1_->get_current_position();
  const float dir       = (target >= setpoint) ? 1.0f : -1.0f;
  const float step      = dir * move_vel_ * dt;

  RCLCPP_INFO(logger, "Homing: driving to %.4f rad", (double)target);
  while (std::chrono::steady_clock::now() < deadline) {
    bool at_goal = std::abs(target - setpoint) <= std::abs(step);
    setpoint = at_goal ? target : setpoint + step;
    float vff = at_goal ? 0.0f : dir * move_vel_;  // velocity feed-forward while moving
    axis1_->MIT_Control(setpoint, vff, hold_kp_, hold_kd_, 0.0f);

    float pos = axis1_->get_current_position();
    if (at_goal && std::abs(pos - target) < 0.02f) break;  // ~1 deg tolerance
    std::this_thread::sleep_for(20ms);
  }
  RCLCPP_INFO(logger, "Homing: arrived at %.4f rad", (double)target);
}

bool HomingController::setArmControllerActive(bool active)
{
  using SwitchController = controller_manager_msgs::srv::SwitchController;
  auto logger = node_->get_logger();
  const char * verb = active ? "activate" : "deactivate";

  if (!switch_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_WARN(logger, "Homing: switch_controller unavailable — cannot %s '%s'",
                verb, managed_controller_.c_str());
    return false;
  }

  auto req = std::make_shared<SwitchController::Request>();
  if (active) req->activate_controllers   = {managed_controller_};
  else        req->deactivate_controllers = {managed_controller_};
  req->strictness    = SwitchController::Request::BEST_EFFORT;
  req->activate_asap = true;

  auto future = switch_client_->async_send_request(req);

  // Spin the client node on a throwaway executor (we are inside node_'s
  // executor callback, so node_ itself must not be spun here).
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(client_node_);
  if (exec.spin_until_future_complete(future, std::chrono::seconds(5)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Homing: switch_controller call to %s '%s' timed out",
                 verb, managed_controller_.c_str());
    return false;
  }

  const bool ok = future.get()->ok;
  RCLCPP_INFO(logger, "Homing: %s '%s' → %s", verb, managed_controller_.c_str(),
              ok ? "ok" : "FAILED");
  return ok;
}

// ----------------------------------------------------------------------------
// Two-sided edge-approach homing:
//   Pass 1 (dir +1): slow approach to edge A           -> P1
//   Overshoot:       continue +1 by overshoot_angle_   (clear the magnet)
//   Pass 2 (dir -1): slow approach to edge B           -> P2
//   midpoint = (P1+P2)/2, zero_target = midpoint + zero_offset_
// ----------------------------------------------------------------------------
void HomingController::runHomingSequence(
  std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
{
  auto logger = node_->get_logger();
  RCLCPP_INFO(logger, "===== HOMING SEQUENCE START =====");

  // Suppress ros2_control write() for axis 1 while homing
  active_.store(true);

  // Pause the arm controller so it neither fights the sequence nor snaps the
  // axis back to its stale pre-homing setpoint once write() resumes.
  const bool controller_paused = setArmControllerActive(false);

  // Run the whole sequence in RAW encoder coordinates: clear any offset from a
  // previous homing run. Otherwise get_current_position() would be offset-
  // corrected while set_home_offset() below expects a raw value, which on a
  // second run collapses the offset toward 0 (axis jumps to the raw position).
  axis1_->set_home_offset(0.0f);

  // Ensure the motor is enabled in position-control mode
  axis1_->Set_Axis_State(8);
  axis1_->Set_Controller_Mode(3, 1);
  std::this_thread::sleep_for(300ms);

  auto fail = [&](const std::string & where) {
    if (controller_paused) setArmControllerActive(true);  // restore on abort
    active_.store(false);
    resp->success = false;
    resp->message = "Homing failed (" + where + "): " + fail_reason_;
    RCLCPP_ERROR(logger, "%s", resp->message.c_str());
  };

  // Sanity range check: refuse to home if the axis is wound up far from its
  // operating range. Beyond this the sweep could also exceed the ±12.5 rad MIT
  // position field. Checked in raw coordinates (offset was just reset to 0).
  const float start_pos = axis1_->get_current_position();
  if (std::abs(start_pos) > max_start_angle_) {
    char buf[128];
    std::snprintf(buf, sizeof(buf), "axis at %.3f rad, outside allowed start range ±%.3f rad",
                  (double)start_pos, (double)max_start_angle_);
    fail_reason_ = buf;
    fail("range check");
    return;
  }

  // If we start already on the magnet, the first edge detection would be
  // mid-magnet (undefined). Back off OPPOSITE to the Pass 1 sweep direction
  // to clear the magnet first.
  if (isOnMagnet()) {
    float back_target = axis1_->get_current_position() - search_dir_ * overshoot_angle_;
    RCLCPP_INFO(logger, "Homing: on magnet at start — backing off to %.4f rad before Pass 1",
                (double)back_target);
    driveToPosition(back_target);
    std::this_thread::sleep_for(300ms);
  }

  // Pass 1: approach edge A in search_dir_ direction
  float p1 = findHomingEdge(search_dir_);
  if (std::isnan(p1)) { fail("pass 1"); return; }
  std::this_thread::sleep_for(300ms);

  // Overshoot: move past the magnet in the SAME direction so pass 2 approaches
  // edge B cleanly from the far side.
  driveToPosition(p1 + search_dir_ * overshoot_angle_);
  std::this_thread::sleep_for(300ms);

  // Pass 2: approach edge B from the opposite direction
  float p2 = findHomingEdge(-search_dir_);
  if (std::isnan(p2)) { fail("pass 2"); return; }
  std::this_thread::sleep_for(300ms);

  // Midpoint of the two switching edges + optional offset = desired zero
  float center      = (p1 + p2) / 2.0f;
  float zero_target = center + zero_offset_;
  RCLCPP_INFO(logger,
    "Homing: P1=%.4f  P2=%.4f  center=%.4f  offset=%.4f  → zero_target=%.4f",
    (double)p1, (double)p2, (double)center, (double)zero_offset_, (double)zero_target);

  driveToPosition(zero_target);
  std::this_thread::sleep_for(300ms);

  // Zero via software home offset. Set_Linear_Count(0) does NOT persist on the
  // GDS68's absolute encoder (the absolute reading overwrites it each cycle), so
  // we instead tell the driver that zero_target (raw) maps to joint 0. From here
  // get_current_position() reports ~0 and MIT_Control(0) holds physical home.
  axis1_->set_home_offset(zero_target);
  RCLCPP_INFO(logger, "Homing: home offset set — joint 0 = %.4f rad (raw)", (double)zero_target);
  std::this_thread::sleep_for(100ms);

  // Let the hardware interface sync its joint_1 state/command back to 0
  if (on_zeroed_) on_zeroed_();

  // Re-enable write() for axis 1, THEN reactivate the arm controller. On
  // activation the JTC re-reads the current state (joint_1 = 0) and holds
  // there, instead of driving back to the pre-homing target.
  active_.store(false);
  if (controller_paused) setArmControllerActive(true);

  resp->success = true;
  resp->message = "Homing complete — axis 1 at zero";
  RCLCPP_INFO(logger, "===== HOMING COMPLETE =====");
}

}  // namespace r0192_hardware
