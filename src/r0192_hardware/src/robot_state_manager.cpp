// ============================================================================
// robot_state_manager.cpp
//
// Central, authoritative state machine for the R0192 arm (single source of
// truth). Five mutually exclusive states:
//
//   DISABLED  motors off,      arm_controller inactive, servo paused
//   HOLD      motors on,       arm_controller inactive, servo paused
//   JOG       motors on,       arm_controller active,   servo running
//   MOVEIT    motors on,       arm_controller active,   servo paused
//   HOMING    homing sequence running (HomingController manages arm_controller)
//
// JOG / MOVEIT / HOMING are only reachable from HOLD (servos must be on). The
// hard gate is the arm_controller: in HOLD it is deactivated, so move_group's
// FollowJointTrajectory action server is down and MoveIt cannot move the arm;
// the hardware still holds position because write() keeps sending MIT_Control
// with the last setpoint while motors are enabled.
//
// Transitions call the existing services:
//   /robot_enable                       (std_srvs/SetBool)        motor torque
//   /controller_manager/switch_controller (SwitchController)      arm_controller
//   /servo_node/pause_servo             (std_srvs/SetBool)        JOG
//   /homing                             (std_srvs/Trigger)        homing
//
// Emergency stop + reset (front door for clients):
//   /e_stop       (std_srvs/Trigger)  forces DISABLED from ANY state, calls the
//                 hardware /robot_estop (latching driver torque-cut). After an
//                 e-stop the drivers are latched in a fault state.
//   /robot_reset  (std_srvs/Trigger)  calls hardware /robot_clear_faults to
//                 un-latch; required before DISABLED->HOLD works again.
//
// Concurrency: executor-thread callbacks (handleSetState/handleEStop/handleReset)
// are serialized by the single-threaded spin of this node and share client_node_.
// The homing worker runs on a SEPARATE thread, so it uses its own homing_node_ —
// rclcpp forbids spinning the same node from two executors concurrently, which
// would otherwise happen when /e_stop arrives mid-homing.
//
// The node owns the state, publishes it latched on /robot_state, and exposes
// /set_robot_state for clients (RViz JogPanel, future r0192_remote web UI).
// ============================================================================

#include <rclcpp/rclcpp.hpp>

#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <r0192_interfaces/msg/robot_state.hpp>
#include <r0192_interfaces/srv/set_robot_state.hpp>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

using namespace std::chrono_literals;

namespace
{
using RobotState     = r0192_interfaces::msg::RobotState;
using SetRobotState  = r0192_interfaces::srv::SetRobotState;
using SetBool        = std_srvs::srv::SetBool;
using Trigger        = std_srvs::srv::Trigger;
using SwitchController = controller_manager_msgs::srv::SwitchController;

const char * stateName(uint8_t s)
{
  switch (s) {
    case RobotState::DISABLED: return "DISABLED";
    case RobotState::HOLD:     return "HOLD";
    case RobotState::JOG:      return "JOG";
    case RobotState::MOVEIT:   return "MOVEIT";
    case RobotState::HOMING:   return "HOMING";
    default:                   return "UNKNOWN";
  }
}
}  // namespace

class RobotStateManager : public rclcpp::Node
{
public:
  RobotStateManager() : rclcpp::Node("r0192_state_manager")
  {
    managed_controller_ = declare_parameter("managed_controller", std::string("arm_controller"));

    // Latched (transient_local) so late subscribers get the current state.
    auto qos = rclcpp::QoS(1).transient_local();
    state_pub_ = create_publisher<RobotState>("/robot_state", qos);

    // Executor-thread callbacks (set_state / e_stop / reset) share client_node_;
    // they never run concurrently under the single-threaded spin in main().
    client_node_   = std::make_shared<rclcpp::Node>("r0192_state_manager_client");
    enable_client_ = client_node_->create_client<SetBool>("/robot_enable");
    pause_client_  = client_node_->create_client<SetBool>("/servo_node/pause_servo");
    switch_client_ = client_node_->create_client<SwitchController>(
      "/controller_manager/switch_controller");
    estop_client_  = client_node_->create_client<Trigger>("/robot_estop");
    reset_client_  = client_node_->create_client<Trigger>("/robot_clear_faults");

    // The homing worker runs on its own thread, so it gets its own node + clients
    // to avoid spinning client_node_ concurrently with an executor-thread call.
    homing_node_          = std::make_shared<rclcpp::Node>("r0192_state_manager_homing_client");
    homing_client_        = homing_node_->create_client<Trigger>("/homing");
    homing_switch_client_ = homing_node_->create_client<SwitchController>(
      "/controller_manager/switch_controller");

    service_ = create_service<SetRobotState>(
      "/set_robot_state",
      [this](const std::shared_ptr<SetRobotState::Request> req,
             std::shared_ptr<SetRobotState::Response> resp) { handleSetState(req, resp); });

    // Emergency stop: forces DISABLED from ANY state (unlike the normal
    // DISABLED<-HOLD transition). Always available. Calls the hardware
    // /robot_estop (latching driver torque-cut) — the drivers stay trapped in a
    // fault state until /robot_reset clears them.
    estop_service_ = create_service<Trigger>(
      "/e_stop",
      [this](const std::shared_ptr<Trigger::Request> req,
             std::shared_ptr<Trigger::Response> resp) { handleEStop(req, resp); });

    // Reset after e-stop: clears the latched driver faults (hardware
    // /robot_clear_faults) so the motors can be enabled again. Stays in DISABLED.
    reset_service_ = create_service<Trigger>(
      "/robot_reset",
      [this](const std::shared_ptr<Trigger::Request> req,
             std::shared_ptr<Trigger::Response> resp) { handleReset(req, resp); });

    RCLCPP_INFO(get_logger(),
      "Robot state manager ready (/set_robot_state, /robot_state, /e_stop, /robot_reset)");
  }

  ~RobotStateManager() override
  {
    if (homing_thread_.joinable()) homing_thread_.join();
  }

  // Reconcile the real stack with the start state: the controller spawner brings
  // arm_controller up active, but the hardware starts de-energized (DISABLED),
  // so deactivate it once at startup. Tolerant of controller_manager not being
  // up yet. Called from main after the executor is ready.
  void initialize()
  {
    std::string msg;
    if (!callSwitch(switch_client_, client_node_, false, msg)) {
      RCLCPP_WARN(get_logger(),
        "Startup: could not deactivate '%s' (%s) — will reconcile on first transition",
        managed_controller_.c_str(), msg.c_str());
    }
    std::lock_guard<std::mutex> lock(mtx_);
    current_state_ = RobotState::DISABLED;
    publishStateLocked("initialised — torque-free");
  }

private:
  // --------------------------------------------------------------------------
  // Service entry point
  // --------------------------------------------------------------------------
  void handleSetState(const std::shared_ptr<SetRobotState::Request> req,
                      std::shared_ptr<SetRobotState::Response> resp)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    const uint8_t target = req->requested_state;

    if (busy_) {
      reject(resp, "busy — homing or a transition is in progress");
      return;
    }
    if (target > RobotState::HOMING) {
      reject(resp, "unknown requested state");
      return;
    }
    if (target == current_state_) {
      resp->success = true;
      resp->message = std::string("already in ") + stateName(target);
      resp->current_state = current_state_;
      return;
    }

    std::string msg;
    bool ok = false;

    switch (current_state_) {
      case RobotState::DISABLED:
        if (target == RobotState::HOLD) {
          if (estop_latched_) {
            reject(resp, "E-STOP latched — call /robot_reset before enabling");
            return;
          }
          ok = callSetBool(enable_client_, true, "/robot_enable", msg);
        } else {
          msg = "enable servos (HOLD) before JOG/MOVEIT/HOMING";
        }
        break;

      case RobotState::HOLD:
        if (target == RobotState::DISABLED) {
          ok = callSetBool(enable_client_, false, "/robot_enable", msg);
        } else if (target == RobotState::MOVEIT) {
          ok = callSwitch(switch_client_, client_node_, true, msg);
        } else if (target == RobotState::JOG) {
          ok = enterJog(msg);
        } else if (target == RobotState::HOMING) {
          startHoming();              // async; sets state + returns immediately
          resp->success = true;
          resp->message = "homing started";
          resp->current_state = current_state_;
          return;
        } else {
          msg = "invalid transition from HOLD";
        }
        break;

      case RobotState::JOG:
        if (target == RobotState::HOLD) ok = exitJog(msg);
        else msg = "return JOG to HOLD before changing state";
        break;

      case RobotState::MOVEIT:
        if (target == RobotState::HOLD) ok = callSwitch(switch_client_, client_node_, false, msg);
        else msg = "return MOVEIT to HOLD before changing state";
        break;

      default:  // HOMING handled by busy_ above
        msg = "homing in progress";
        break;
    }

    if (ok) {
      current_state_ = target;
      resp->success = true;
      resp->message = std::string("-> ") + stateName(target);
      publishStateLocked(resp->message);
    } else {
      resp->success = false;
      resp->message = msg;
    }
    resp->current_state = current_state_;
  }

  // --------------------------------------------------------------------------
  // Emergency stop — force DISABLED from any state
  // --------------------------------------------------------------------------
  void handleEStop(const std::shared_ptr<Trigger::Request>,
                   std::shared_ptr<Trigger::Response> resp)
  {
    // Cut torque at the driver level FIRST (latching), before touching the
    // mutex — this is the safety-critical action and must not wait on bookkeeping.
    std::string emsg;
    const bool cut = callTrigger(estop_client_, "/robot_estop", emsg);

    std::lock_guard<std::mutex> lock(mtx_);
    estop_latched_ = true;

    if (busy_) {
      // Homing in progress: the homing thread owns axis 1 and drives it directly
      // (bypassing motors_enabled_), so we cannot tear it down here. Flag it to
      // land in DISABLED instead of HOLD; driver torque is already cut.
      // NOTE: v1 does NOT physically interrupt an in-flight homing sweep — that
      // needs a HomingController abort hook (future work).
      estop_ = true;
      resp->success = true;
      resp->message = "E-STOP during homing — torque cut, will land in DISABLED (reset required)";
      RCLCPP_ERROR(get_logger(), "%s", resp->message.c_str());
      return;
    }

    // Full safe teardown from whatever state we were in (best effort).
    std::string msg;
    callSetBool(pause_client_, true, "/servo_node/pause_servo", msg);
    callSwitch(switch_client_, client_node_, false, msg);
    current_state_ = RobotState::DISABLED;
    publishStateLocked("E-STOP — torque cut, DISABLED (reset required)");
    resp->success = cut;
    resp->message = cut ? "E-STOP — drivers latched, call /robot_reset to recover"
                        : ("E-STOP — torque cut best-effort (" + emsg + ")");
    RCLCPP_ERROR(get_logger(), "E-STOP triggered — state forced to DISABLED");
  }

  // --------------------------------------------------------------------------
  // Reset — clear latched driver faults after an e-stop
  // --------------------------------------------------------------------------
  void handleReset(const std::shared_ptr<Trigger::Request>,
                   std::shared_ptr<Trigger::Response> resp)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (busy_) {
      resp->success = false;
      resp->message = "busy — cannot reset during homing/transition";
      return;
    }
    std::string msg;
    const bool ok = callTrigger(reset_client_, "/robot_clear_faults", msg);
    if (ok) estop_latched_ = false;
    // Reset leaves us de-energized; ensure we are in DISABLED.
    current_state_ = RobotState::DISABLED;
    publishStateLocked(ok ? "reset — faults cleared, DISABLED" : ("reset failed: " + msg));
    resp->success = ok;
    resp->message = ok ? "drivers reset — ready to enable" : msg;
  }

  // --------------------------------------------------------------------------
  // Composite transitions
  // --------------------------------------------------------------------------
  bool enterJog(std::string & msg)
  {
    if (!callSwitch(switch_client_, client_node_, true, msg)) return false;
    if (!callSetBool(pause_client_, false, "/servo_node/pause_servo", msg)) {
      std::string rb;
      callSwitch(switch_client_, client_node_, false, rb);   // roll back the controller activation
      return false;
    }
    return true;
  }

  bool exitJog(std::string & msg)
  {
    // Pause servo first so it hands the controller back, then deactivate it.
    const bool paused = callSetBool(pause_client_, true, "/servo_node/pause_servo", msg);
    const bool deact  = callSwitch(switch_client_, client_node_, false, msg);
    return paused && deact;
  }

  void startHoming()
  {
    // Safe to join here: reaching this point means busy_ is false, so any prior
    // homing thread has already passed its locked tail section and is exiting.
    if (homing_thread_.joinable()) homing_thread_.join();

    busy_ = true;
    current_state_ = RobotState::HOMING;
    publishStateLocked("homing started");

    homing_thread_ = std::thread([this]() {
      std::string msg;
      const bool ok = callHoming(msg);
      // The HomingController reactivates arm_controller when it finishes; HOLD
      // expects it inactive, so deactivate it again (on the homing node).
      std::string smsg;
      callSwitch(homing_switch_client_, homing_node_, false, smsg);

      std::lock_guard<std::mutex> lock(mtx_);
      busy_ = false;
      if (estop_) {
        estop_ = false;
        current_state_ = RobotState::DISABLED;
        publishStateLocked("homing aborted by E-STOP — DISABLED (reset required)");
      } else {
        current_state_ = RobotState::HOLD;
        publishStateLocked(ok ? ("homing ok: " + msg) : ("homing failed: " + msg));
      }
    });
  }

  // --------------------------------------------------------------------------
  // Service-call helpers. `node` is the node spun while waiting on the future;
  // it MUST be the node the client was created on (see concurrency note above).
  // --------------------------------------------------------------------------
  template <typename FutureT>
  bool waitFuture(const rclcpp::Node::SharedPtr & node, FutureT & future,
                  std::chrono::nanoseconds timeout)
  {
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    return exec.spin_until_future_complete(future, timeout) ==
           rclcpp::FutureReturnCode::SUCCESS;
  }

  bool callSetBool(rclcpp::Client<SetBool>::SharedPtr cli, bool data,
                   const char * name, std::string & msg)
  {
    if (!cli->wait_for_service(1s)) { msg = std::string(name) + " unavailable"; return false; }
    auto req = std::make_shared<SetBool::Request>();
    req->data = data;
    auto fut = cli->async_send_request(req);
    if (!waitFuture(client_node_, fut, 5s)) { msg = std::string(name) + " call timed out"; return false; }
    auto resp = fut.get();
    if (!resp->success) { msg = resp->message.empty() ? (std::string(name) + " reported failure")
                                                       : resp->message; return false; }
    return true;
  }

  bool callTrigger(rclcpp::Client<Trigger>::SharedPtr cli, const char * name, std::string & msg)
  {
    if (!cli->wait_for_service(1s)) { msg = std::string(name) + " unavailable"; return false; }
    auto req = std::make_shared<Trigger::Request>();
    auto fut = cli->async_send_request(req);
    if (!waitFuture(client_node_, fut, 5s)) { msg = std::string(name) + " call timed out"; return false; }
    auto resp = fut.get();
    if (!resp->success) { msg = resp->message.empty() ? (std::string(name) + " reported failure")
                                                       : resp->message; return false; }
    return true;
  }

  bool callSwitch(rclcpp::Client<SwitchController>::SharedPtr cli,
                  const rclcpp::Node::SharedPtr & node, bool activate, std::string & msg)
  {
    if (!cli->wait_for_service(1s)) {
      msg = "switch_controller unavailable"; return false;
    }
    auto req = std::make_shared<SwitchController::Request>();
    if (activate) req->activate_controllers   = {managed_controller_};
    else          req->deactivate_controllers = {managed_controller_};
    req->strictness    = SwitchController::Request::BEST_EFFORT;
    req->activate_asap = true;
    auto fut = cli->async_send_request(req);
    if (!waitFuture(node, fut, 5s)) { msg = "switch_controller timed out"; return false; }
    if (!fut.get()->ok) { msg = std::string("could not ") + (activate ? "activate '" : "deactivate '")
                                + managed_controller_ + "'"; return false; }
    return true;
  }

  bool callHoming(std::string & msg)
  {
    if (!homing_client_->wait_for_service(1s)) {
      msg = "/homing unavailable (axis 1 present?)"; return false;
    }
    auto req = std::make_shared<Trigger::Request>();
    auto fut = homing_client_->async_send_request(req);
    if (!waitFuture(homing_node_, fut, 180s)) { msg = "/homing timed out"; return false; }
    auto resp = fut.get();
    msg = resp->message;
    return resp->success;
  }

  // --------------------------------------------------------------------------
  void reject(std::shared_ptr<SetRobotState::Response> & resp, const std::string & why)
  {
    resp->success = false;
    resp->message = why;
    resp->current_state = current_state_;
  }

  void publishStateLocked(const std::string & status)
  {
    RobotState msg;
    msg.state = current_state_;
    msg.status = status;
    state_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "State -> %s (%s)", stateName(current_state_), status.c_str());
  }

  // --- State ---
  std::mutex mtx_;
  uint8_t current_state_{RobotState::DISABLED};
  bool busy_{false};
  bool estop_{false};          // set by /e_stop while homing; homing thread lands in DISABLED
  bool estop_latched_{false};  // drivers latched after e-stop; needs /robot_reset
  std::string managed_controller_;

  // --- ROS ---
  rclcpp::Publisher<RobotState>::SharedPtr state_pub_;
  rclcpp::Service<SetRobotState>::SharedPtr service_;
  rclcpp::Service<Trigger>::SharedPtr estop_service_;
  rclcpp::Service<Trigger>::SharedPtr reset_service_;

  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Client<SetBool>::SharedPtr enable_client_;
  rclcpp::Client<SetBool>::SharedPtr pause_client_;
  rclcpp::Client<SwitchController>::SharedPtr switch_client_;
  rclcpp::Client<Trigger>::SharedPtr estop_client_;
  rclcpp::Client<Trigger>::SharedPtr reset_client_;

  rclcpp::Node::SharedPtr homing_node_;
  rclcpp::Client<Trigger>::SharedPtr homing_client_;
  rclcpp::Client<SwitchController>::SharedPtr homing_switch_client_;

  std::thread homing_thread_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotStateManager>();
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
