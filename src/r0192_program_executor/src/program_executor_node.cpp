// ============================================================================
// program_executor_node.cpp
//
// Backend of the R0192 program system (engineering in VS Code, operations in
// the RViz ProgramPanel). Executes YAML programs step by step and is the only
// component with program business logic.
//
//   /execute_program   (r0192_interfaces/action/ExecuteProgram)
//     Goal carries a file path (absolute, or relative to the programs_dir
//     parameter). Points are loaded from points_file. One goal at a time.
//
// State handling: exclusively via /set_robot_state (robot_state_manager).
//   - Before the first step the executor requests MOVEIT (rejected by the
//     manager unless the arm is in HOLD -> the goal aborts with that reason).
//   - On finish / failure / cancel it requests HOLD again, but only if the
//     state is still MOVEIT — after an /e_stop the state is DISABLED and a
//     HOLD request would mean "enable motors", which is not ours to decide.
//
// Execution model (v1, sequential):
//   - move_j: MoveGroupInterface plan + execute (joint or pose target),
//     velocity/acceleration as MoveIt scaling factors in (0, 1].
//   - wait:   sleep in small ticks so cancel stays responsive.
//   - move_l: schema-valid but rejected at goal time (Phase 6).
//
// Cancel: handle_cancel calls MoveGroupInterface::stop(), which makes a
// blocking execute() return; the worker then reports CANCELED and returns
// the state to HOLD.
//
// Threading: the action callbacks run on the main spin; each accepted goal
// runs in its own worker thread (goals are serialized via busy_). The
// /set_robot_state futures complete because the main spin keeps serving this
// node while the worker waits. MoveGroupInterface spins its own internal
// callback thread, so its blocking calls are safe here.
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>

#include <r0192_interfaces/action/execute_program.hpp>
#include <r0192_interfaces/msg/robot_state.hpp>
#include <r0192_interfaces/srv/set_robot_state.hpp>

#include "r0192_program_executor/program_loader.hpp"

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>

using namespace std::chrono_literals;

namespace r0192_program_executor
{

using ExecuteProgram = r0192_interfaces::action::ExecuteProgram;
using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteProgram>;
using RobotStateMsg = r0192_interfaces::msg::RobotState;
using SetRobotState = r0192_interfaces::srv::SetRobotState;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

namespace
{
std::string expandUser(std::string path)
{
  if (!path.empty() && path[0] == '~') {
    if (const char * home = std::getenv("HOME")) {
      path = std::string(home) + path.substr(1);
    }
  }
  return path;
}
}  // namespace

class ProgramExecutor : public rclcpp::Node
{
public:
  ProgramExecutor() : rclcpp::Node("r0192_program_executor")
  {
    programs_dir_ = expandUser(
      declare_parameter("programs_dir", std::string("~/roboticarm_r0192_ws/programs")));
    points_file_ = expandUser(declare_parameter("points_file", std::string("")));
    if (points_file_.empty()) {
      points_file_ = (std::filesystem::path(programs_dir_) / "points.yaml").string();
    }
    planning_group_ = declare_parameter("planning_group", std::string("r0192_arm"));

    state_client_ = create_client<SetRobotState>("/set_robot_state");

    // Mirror the authoritative state (latched) so we never request HOLD from a
    // state we do not own (e.g. DISABLED after an /e_stop mid-program).
    auto qos = rclcpp::QoS(1).transient_local();
    state_sub_ = create_subscription<RobotStateMsg>(
      "/robot_state", qos,
      [this](const RobotStateMsg & msg) { robot_state_ = msg.state; });

    server_ = rclcpp_action::create_server<ExecuteProgram>(
      this, "/execute_program",
      [this](const rclcpp_action::GoalUUID &, ExecuteProgram::Goal::ConstSharedPtr goal) {
        return handleGoal(goal);
      },
      [this](const std::shared_ptr<GoalHandle> &) { return handleCancel(); },
      [this](const std::shared_ptr<GoalHandle> & gh) { handleAccepted(gh); });

    RCLCPP_INFO(get_logger(),
      "Program executor ready (/execute_program) — programs_dir: %s, points: %s",
      programs_dir_.c_str(), points_file_.c_str());
  }

  ~ProgramExecutor() override
  {
    if (worker_.joinable()) worker_.join();
  }

private:
  // --------------------------------------------------------------------------
  // Action callbacks (main spin)
  // --------------------------------------------------------------------------
  rclcpp_action::GoalResponse handleGoal(ExecuteProgram::Goal::ConstSharedPtr goal)
  {
    if (goal->program_path.empty()) {
      RCLCPP_WARN(get_logger(), "Goal rejected: empty program_path");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (busy_) {
      RCLCPP_WARN(get_logger(), "Goal rejected: a program is already running");
      return rclcpp_action::GoalResponse::REJECT;
    }
    busy_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel()
  {
    RCLCPP_INFO(get_logger(), "Cancel requested — stopping current motion");
    // Breaks a blocking plan()/execute() in the worker; the worker then sees
    // is_canceling() and finishes the goal as CANCELED.
    if (move_group_) move_group_->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandle> & goal_handle)
  {
    // The previous worker (if any) has finished — busy_ guards concurrency.
    if (worker_.joinable()) worker_.join();
    worker_ = std::thread([this, goal_handle]() {
      executeProgram(goal_handle);
      busy_ = false;
    });
  }

  // --------------------------------------------------------------------------
  // Worker thread: load -> MOVEIT -> steps -> HOLD
  // --------------------------------------------------------------------------
  void executeProgram(const std::shared_ptr<GoalHandle> & goal_handle)
  {
    auto result = std::make_shared<ExecuteProgram::Result>();
    result->steps_completed = 0;

    const std::string path = resolveProgramPath(goal_handle->get_goal()->program_path);
    publishFeedback(goal_handle, 0, 0, nullptr, ExecuteProgram::Feedback::STATUS_LOADING);

    Program program;
    PointMap points;
    try {
      points = loadPoints(points_file_);
      program = loadProgram(path);
      crossValidate(program, points);
      for (const auto & s : program.steps) {
        if (s.type == "move_l") {
          throw std::runtime_error("move_l is not implemented yet (plan phase 6)");
        }
      }
    } catch (const std::exception & e) {
      result->message = e.what();
      goal_handle->abort(result);
      RCLCPP_ERROR(get_logger(), "Program rejected: %s", e.what());
      return;
    }

    std::string err;
    if (!ensureMoveGroup(err)) {
      result->message = "MoveIt unavailable: " + err;
      goal_handle->abort(result);
      RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
      return;
    }

    if (!setRobotState(RobotStateMsg::MOVEIT, err)) {
      result->message = "cannot enter MOVEIT: " + err;
      goal_handle->abort(result);
      RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "Executing program '%s' (%zu steps) from %s",
                program.name.c_str(), program.steps.size(), path.c_str());

    bool ok = true;
    bool cancelled = false;
    for (std::size_t i = 0; i < program.steps.size(); ++i) {
      if (goal_handle->is_canceling()) {
        cancelled = true;
        break;
      }
      const Step & step = program.steps[i];
      if (step.type == "wait") {
        ok = runWait(goal_handle, i, program, cancelled);
      } else {
        ok = runMoveJ(goal_handle, i, program, points.at(step.target), cancelled, err);
      }
      if (!ok || cancelled) break;
      result->steps_completed = static_cast<uint32_t>(i + 1);
    }

    // Always hand control back: MOVEIT -> HOLD (only if we still own MOVEIT —
    // after an /e_stop the manager has forced DISABLED and HOLD would mean
    // re-enabling the motors).
    std::string hold_note;
    if (robot_state_ == RobotStateMsg::MOVEIT) {
      std::string hold_err;
      if (!setRobotState(RobotStateMsg::HOLD, hold_err)) {
        hold_note = " (warning: could not return to HOLD: " + hold_err + ")";
      }
    } else {
      hold_note = " (state no longer MOVEIT — leaving state untouched)";
    }

    if (cancelled) {
      result->message = "cancelled after " + std::to_string(result->steps_completed) +
                        " of " + std::to_string(program.steps.size()) + " steps" + hold_note;
      goal_handle->canceled(result);
      RCLCPP_WARN(get_logger(), "Program '%s' %s", program.name.c_str(),
                  result->message.c_str());
    } else if (!ok) {
      result->message = "step " + std::to_string(result->steps_completed + 1) +
                        " failed: " + err + hold_note;
      goal_handle->abort(result);
      RCLCPP_ERROR(get_logger(), "Program '%s' aborted: %s", program.name.c_str(),
                   result->message.c_str());
    } else {
      result->success = true;
      result->message = "program finished (" + std::to_string(result->steps_completed) +
                        " steps)" + hold_note;
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "Program '%s' finished", program.name.c_str());
    }
  }

  // --------------------------------------------------------------------------
  // Steps
  // --------------------------------------------------------------------------
  bool runWait(const std::shared_ptr<GoalHandle> & goal_handle, std::size_t index,
               const Program & program, bool & cancelled)
  {
    const Step & step = program.steps[index];
    publishFeedback(goal_handle, index, program.steps.size(), &step,
                    ExecuteProgram::Feedback::STATUS_WAITING);
    const auto end = std::chrono::steady_clock::now() +
                     std::chrono::duration<double>(step.duration);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < end) {
      if (goal_handle->is_canceling()) {
        cancelled = true;
        return true;
      }
      std::this_thread::sleep_for(50ms);
    }
    return true;
  }

  bool runMoveJ(const std::shared_ptr<GoalHandle> & goal_handle, std::size_t index,
                const Program & program, const Point & point, bool & cancelled,
                std::string & err)
  {
    const Step & step = program.steps[index];
    publishFeedback(goal_handle, index, program.steps.size(), &step,
                    ExecuteProgram::Feedback::STATUS_PLANNING);

    move_group_->setStartStateToCurrentState();
    move_group_->setMaxVelocityScalingFactor(step.velocity);
    move_group_->setMaxAccelerationScalingFactor(step.acceleration);

    if (point.type == Point::Type::kJoint) {
      std::map<std::string, double> target;
      for (std::size_t i = 0; i < joint_names_.size(); ++i) {
        target[joint_names_[i]] = point.joints[i];
      }
      move_group_->setJointValueTarget(target);
    } else {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = point.frame;
      ps.pose = point.pose;
      move_group_->setPoseTarget(ps);
    }

    MoveGroupInterface::Plan plan;
    auto code = move_group_->plan(plan);
    move_group_->clearPoseTargets();
    if (goal_handle->is_canceling()) {
      cancelled = true;
      return true;
    }
    if (code != moveit::core::MoveItErrorCode::SUCCESS) {
      err = "planning to '" + step.target + "' failed (MoveIt error " +
            std::to_string(code.val) + ")";
      return false;
    }

    publishFeedback(goal_handle, index, program.steps.size(), &step,
                    ExecuteProgram::Feedback::STATUS_MOVING);
    code = move_group_->execute(plan);
    if (goal_handle->is_canceling()) {
      cancelled = true;
      return true;
    }
    if (code != moveit::core::MoveItErrorCode::SUCCESS) {
      err = "execution of '" + step.target + "' failed (MoveIt error " +
            std::to_string(code.val) + ")";
      return false;
    }
    return true;
  }

  // --------------------------------------------------------------------------
  // Helpers
  // --------------------------------------------------------------------------
  bool ensureMoveGroup(std::string & err)
  {
    if (move_group_) return true;
    try {
      move_group_ = std::make_shared<MoveGroupInterface>(
        shared_from_this(), planning_group_, std::shared_ptr<tf2_ros::Buffer>(),
        rclcpp::Duration::from_seconds(15.0));
    } catch (const std::exception & e) {
      err = e.what();
      return false;
    }
    joint_names_ = move_group_->getJointNames();
    if (joint_names_.size() != 6) {
      err = "planning group '" + planning_group_ + "' has " +
            std::to_string(joint_names_.size()) + " active joints, expected 6";
      move_group_.reset();
      return false;
    }
    return true;
  }

  bool setRobotState(uint8_t state, std::string & err)
  {
    if (!state_client_->wait_for_service(2s)) {
      err = "/set_robot_state unavailable (robot_state_manager running?)";
      return false;
    }
    auto req = std::make_shared<SetRobotState::Request>();
    req->requested_state = state;
    auto future = state_client_->async_send_request(req);
    // The main spin serves this node, so the future completes here.
    if (future.wait_for(10s) != std::future_status::ready) {
      state_client_->remove_pending_request(future);
      err = "/set_robot_state timed out";
      return false;
    }
    const auto resp = future.get();
    if (!resp->success) {
      err = resp->message;
      return false;
    }
    return true;
  }

  void publishFeedback(const std::shared_ptr<GoalHandle> & goal_handle, std::size_t index,
                       std::size_t total, const Step * step, uint8_t status)
  {
    auto fb = std::make_shared<ExecuteProgram::Feedback>();
    fb->current_step = static_cast<uint32_t>(index);
    fb->total_steps = static_cast<uint32_t>(total);
    fb->status = status;
    if (step) {
      fb->step_type = step->type;
      fb->step_label = step->label();
    }
    goal_handle->publish_feedback(fb);
  }

  std::string resolveProgramPath(const std::string & in) const
  {
    std::filesystem::path p(expandUser(in));
    if (p.is_relative()) p = std::filesystem::path(programs_dir_) / p;
    return p.string();
  }

  // --- Parameters ---
  std::string programs_dir_;
  std::string points_file_;
  std::string planning_group_;

  // --- ROS ---
  rclcpp_action::Server<ExecuteProgram>::SharedPtr server_;
  rclcpp::Client<SetRobotState>::SharedPtr state_client_;
  rclcpp::Subscription<RobotStateMsg>::SharedPtr state_sub_;

  // --- Execution ---
  std::shared_ptr<MoveGroupInterface> move_group_;  // lazy (move_group up later)
  std::vector<std::string> joint_names_;
  std::thread worker_;
  std::atomic<bool> busy_{false};
  std::atomic<uint8_t> robot_state_{RobotStateMsg::DISABLED};
};

}  // namespace r0192_program_executor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<r0192_program_executor::ProgramExecutor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
