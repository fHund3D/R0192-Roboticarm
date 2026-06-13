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
// Execution model:
//   Two step vocabularies and two run modes.
//   - "Stop at each point" (goal.blend == false, default): each move step is
//     planned + executed on its own.
//       move_j: OMPL plan+execute (joint or pose).      ptp: Pilz PTP.
//       move_l: KDL computeCartesianPath (Cartesian).   lin: Pilz LIN.
//       circ:   rejected (needs blend mode).            wait: cancel-responsive sleep.
//     velocity/acceleration (vel/acc) are MoveIt scaling factors in (0, 1].
//   - "Blend through" (goal.blend == true): consecutive move steps are batched
//     into one Pilz MotionSequenceRequest (planner per step, blend_radius=c_dis,
//     last item forced to 0) and sent to the /sequence_move_group action. circ
//     is supported here (via point = interim). wait steps split a sequence.
//
// Cancel: handle_cancel calls MoveGroupInterface::stop() (sequential) and
// cancels the active /sequence_move_group goal (blend). The worker then reports
// CANCELED and returns the state to HOLD.
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
#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit/robot_state/conversions.hpp>

#include <moveit_msgs/action/move_group_sequence.hpp>
#include <moveit_msgs/msg/motion_sequence_request.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <r0192_interfaces/action/execute_program.hpp>
#include <r0192_interfaces/msg/robot_state.hpp>
#include <r0192_interfaces/srv/delete_point.hpp>
#include <r0192_interfaces/srv/list_points.hpp>
#include <r0192_interfaces/srv/set_program_override.hpp>
#include <r0192_interfaces/srv/set_robot_state.hpp>
#include <r0192_interfaces/srv/teach_point.hpp>

#include "r0192_program_executor/program_loader.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

using namespace std::chrono_literals;

namespace r0192_program_executor
{

using ExecuteProgram = r0192_interfaces::action::ExecuteProgram;
using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteProgram>;
using RobotStateMsg = r0192_interfaces::msg::RobotState;
using SetRobotState = r0192_interfaces::srv::SetRobotState;
using TeachPoint = r0192_interfaces::srv::TeachPoint;
using ListPoints = r0192_interfaces::srv::ListPoints;
using DeletePoint = r0192_interfaces::srv::DeletePoint;
using SetProgramOverride = r0192_interfaces::srv::SetProgramOverride;
using Trigger = std_srvs::srv::Trigger;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using MoveGroupSequence = moveit_msgs::action::MoveGroupSequence;

namespace
{
// Arm joints in point order (matches the SRDF group r0192_arm and the
// 'joints:' arrays in points.yaml).
constexpr std::array<const char *, 6> kArmJoints = {
  "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

std::string expandUser(std::string path)
{
  if (!path.empty() && path[0] == '~') {
    if (const char * home = std::getenv("HOME")) {
      path = std::string(home) + path.substr(1);
    }
  }
  return path;
}

bool isMoveStep(const std::string & t)
{
  return t == "move_j" || t == "move_l" || t == "ptp" || t == "lin" || t == "circ";
}

// Pilz planner id for a move step. Legacy aliases map onto their KRL twin
// (move_j ~ ptp, move_l ~ lin).
const char * pilzPlannerId(const std::string & t)
{
  if (t == "lin" || t == "move_l") return "LIN";
  if (t == "circ") return "CIRC";
  return "PTP";   // ptp / move_j
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
    // Link whose pose is captured by /teach_point (TYPE_POSE). Must be the
    // planning group's end-effector link so a taught pose replays exactly.
    pose_reference_link_ =
      declare_parameter("pose_reference_link", std::string("gripper_base"));
    // Max EE distance between consecutive IK samples of a move_l path (m).
    cartesian_eef_step_ = declare_parameter("cartesian_eef_step", 0.005);

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

    // Pilz blend mode (phase 7) sends whole runs of move steps to the
    // /sequence_move_group action (provided by the Pilz MoveGroupSequenceAction
    // capability — added to move_group in moveit.launch.py).
    seq_client_ = rclcpp_action::create_client<MoveGroupSequence>(this, "/sequence_move_group");

    // Dry-run preview: planned trajectories are animated as the RViz ghost on
    // the same topic move_group uses for plan previews.
    display_pub_ = create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      "/display_planned_path", rclcpp::QoS(1));

    // --- Point services (phase 4). All run on the main spin (short file ops);
    //     the joint-state cache below is filled on the same thread, so no lock
    //     is needed. ---
    teach_srv_ = create_service<TeachPoint>(
      "/teach_point",
      [this](const std::shared_ptr<TeachPoint::Request> req,
             std::shared_ptr<TeachPoint::Response> resp) { handleTeach(req, resp); });
    list_srv_ = create_service<ListPoints>(
      "/list_points",
      [this](const std::shared_ptr<ListPoints::Request>,
             std::shared_ptr<ListPoints::Response> resp) { handleList(resp); });
    delete_srv_ = create_service<DeletePoint>(
      "/delete_point",
      [this](const std::shared_ptr<DeletePoint::Request> req,
             std::shared_ptr<DeletePoint::Response> resp) { handleDelete(req, resp); });

    // --- Pause / Resume (phase 5): "pause after current step" — the running
    //     step finishes, then the worker holds before the next one. The state
    //     stays MOVEIT while paused (the goal still owns the state machine). ---
    pause_srv_ = create_service<Trigger>(
      "/pause_program",
      [this](const std::shared_ptr<Trigger::Request>,
             std::shared_ptr<Trigger::Response> resp) {
        if (!busy_) {
          resp->success = false;
          resp->message = "no program running";
          return;
        }
        pause_requested_ = true;
        resp->success = true;
        resp->message = "pausing after the current step";
        RCLCPP_INFO(get_logger(), "%s", resp->message.c_str());
      });
    resume_srv_ = create_service<Trigger>(
      "/resume_program",
      [this](const std::shared_ptr<Trigger::Request>,
             std::shared_ptr<Trigger::Response> resp) {
        if (!busy_ || !pause_requested_) {
          resp->success = false;
          resp->message = "no paused program";
          return;
        }
        pause_requested_ = false;
        resp->success = true;
        resp->message = "resuming";
        RCLCPP_INFO(get_logger(), "%s", resp->message.c_str());
      });

    // --- Speed override (phase 5): multiplies each step's scaling at plan
    //     time -> effective from the NEXT step. Applied value is published
    //     latched on /program_override; UIs mirror that topic. ---
    override_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/program_override", rclcpp::QoS(1).transient_local());
    publishOverride();
    override_srv_ = create_service<SetProgramOverride>(
      "/set_program_override",
      [this](const std::shared_ptr<SetProgramOverride::Request> req,
             std::shared_ptr<SetProgramOverride::Response> resp) {
        const double applied = std::clamp(static_cast<double>(req->override), 0.1, 1.0);
        override_ = applied;
        publishOverride();
        resp->success = true;
        resp->override = static_cast<float>(applied);
        resp->message = "override set to " + std::to_string(applied) +
                        (applied != static_cast<double>(req->override) ? " (clamped)" : "");
        RCLCPP_INFO(get_logger(), "%s", resp->message.c_str());
      });

    // --- Point visualization (phase 6): spheres + name labels for all points
    //     in points.yaml, published latched. Pose points directly; joint
    //     points via FK once the robot model is available (lazy MoveGroup). ---
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/program_points_markers", rclcpp::QoS(1).transient_local());
    publishPointMarkers();

    // Sources for teaching: joint angles from /joint_states, EE pose from TF.
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(10),
      [this](const sensor_msgs::msg::JointState & msg) {
        const std::size_t n = std::min(msg.name.size(), msg.position.size());
        for (std::size_t i = 0; i < n; ++i) joint_pos_[msg.name[i]] = msg.position[i];
      });
    // Dedicated spin thread: the teach handler blocks the main spin while it
    // waits in lookupTransform(timeout), so TF data must arrive on another
    // thread (tf2 errors out otherwise).
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
      *tf_buffer_, static_cast<rclcpp::Node *>(this), true);

    RCLCPP_INFO(get_logger(),
      "Program executor ready (/execute_program, /teach_point, /list_points, "
      "/delete_point, /pause_program, /resume_program, /set_program_override) "
      "— programs_dir: %s, points: %s",
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
    // Sequential: breaks a blocking plan()/execute() in the worker.
    if (auto mg = moveGroup()) mg->stop();
    // Blend mode: cancel the in-flight /sequence_move_group goal (its result
    // future then returns CANCELED). The worker sees is_canceling() either way
    // and finishes the goal as CANCELED.
    std::shared_ptr<rclcpp_action::ClientGoalHandle<MoveGroupSequence>> sg;
    { std::lock_guard<std::mutex> lock(seq_mtx_); sg = seq_goal_; }
    if (sg) seq_client_->async_cancel_goal(sg);
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
    pause_requested_ = false;   // a stale pause never carries into a new goal

    const std::string path = resolveProgramPath(goal_handle->get_goal()->program_path);
    publishFeedback(goal_handle, 0, 0, nullptr, ExecuteProgram::Feedback::STATUS_LOADING);

    Program program;
    PointMap points;
    try {
      points = loadPoints(points_file_);
      program = loadProgram(path);
      crossValidate(program, points);
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

    const bool blend = goal_handle->get_goal()->blend;
    const bool dry_run = goal_handle->get_goal()->dry_run;

    // A real run needs MOVEIT (motors on, arm_controller active) before the first
    // step. A dry run only plans + animates the RViz ghost — it never executes and
    // never changes the robot state, so it skips the transition (works from any state).
    if (!dry_run && !setRobotState(RobotStateMsg::MOVEIT, err)) {
      result->message = "cannot enter MOVEIT: " + err;
      goal_handle->abort(result);
      RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "%s program '%s' (%zu steps) from %s",
                dry_run ? "Simulating" : "Executing",
                program.name.c_str(), program.steps.size(), path.c_str());
    bool ok = true;
    bool cancelled = false;
    std::size_t i = 0;
    while (i < program.steps.size()) {
      // Pause point: between steps ("pause after current step"). The state
      // stays MOVEIT; the JTC holds the last setpoint. Cancel still works.
      if (pause_requested_) {
        RCLCPP_INFO(get_logger(), "Program paused before step %zu/%zu",
                    i + 1, program.steps.size());
        while (rclcpp::ok() && pause_requested_ && !goal_handle->is_canceling()) {
          publishFeedback(goal_handle, i, program.steps.size(), &program.steps[i],
                          ExecuteProgram::Feedback::STATUS_PAUSED);
          std::this_thread::sleep_for(200ms);
        }
        if (!pause_requested_) {
          RCLCPP_INFO(get_logger(), "Program resumed at step %zu", i + 1);
        }
      }
      if (goal_handle->is_canceling()) {
        cancelled = true;
        break;
      }
      const Step & step = program.steps[i];
      if (step.type == "wait") {
        ok = dry_run ? true : runWait(goal_handle, i, program, cancelled);  // skip waits in a preview
        if (ok && !cancelled) result->steps_completed = static_cast<uint32_t>(i + 1);
        ++i;
      } else if (blend || dry_run) {
        // Blend through (or any dry run): the maximal run of consecutive move
        // steps [i, j) goes to the Pilz sequence (wait steps split it). dry_run
        // makes it plan-only + animate; blend controls whether c_dis is applied.
        std::size_t j = i;
        while (j < program.steps.size() && isMoveStep(program.steps[j].type)) ++j;
        ok = runBlendedRun(goal_handle, i, j, program, points, cancelled, err, dry_run, blend);
        if (ok && !cancelled) result->steps_completed = static_cast<uint32_t>(j);
        i = j;
      } else if (step.type == "move_l") {
        ok = runMoveL(goal_handle, i, program, points.at(step.target), cancelled, err);
        if (ok && !cancelled) result->steps_completed = static_cast<uint32_t>(i + 1);
        ++i;
      } else if (step.type == "ptp" || step.type == "lin") {
        ok = runPilz(goal_handle, i, program, points.at(step.target), cancelled, err);
        if (ok && !cancelled) result->steps_completed = static_cast<uint32_t>(i + 1);
        ++i;
      } else if (step.type == "circ") {
        // CIRC needs the via point planned as one arc — only the sequence mode
        // builds that. Reject cleanly in stop-at-each-point mode.
        err = "circ requires blend mode — run the program with blend=true "
              "(\"blend through\") so the via point becomes a Pilz CIRC segment";
        ok = false;
        ++i;
      } else {
        ok = runMoveJ(goal_handle, i, program, points.at(step.target), cancelled, err);
        if (ok && !cancelled) result->steps_completed = static_cast<uint32_t>(i + 1);
        ++i;
      }
      if (!ok || cancelled) break;
    }

    // Always hand control back: MOVEIT -> HOLD (only if we still own MOVEIT —
    // after an /e_stop the manager has forced DISABLED and HOLD would mean
    // re-enabling the motors). A dry run never entered MOVEIT, so it leaves the
    // state alone.
    std::string hold_note;
    if (dry_run) {
      hold_note = " (dry run — state untouched)";
    } else if (robot_state_ == RobotStateMsg::MOVEIT) {
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
    // Speed override: read before every plan, so a change takes effect from
    // the next step. step.* in (0,1] times override in [0.1,1] stays in (0,1].
    const double ov = override_;
    move_group_->setMaxVelocityScalingFactor(step.velocity * ov);
    move_group_->setMaxAccelerationScalingFactor(step.acceleration * ov);

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

  // Linear Cartesian move (phase 6): straight EE line to a pose-type point via
  // computeCartesianPath. Time parameterization happens SERVER-side: the
  // cartesian-path service applies TOTG with the scaling factors from the
  // request and the joint_limits.yaml only move_group has loaded (the client
  // robot model lacks acceleration limits, so client-side TOTG would fail).
  // NOTE: KDL cannot iterate along a line through the wrist singularity
  // (joint_5 = 0) — such programs fail cleanly with a 0%-feasible message.
  bool runMoveL(const std::shared_ptr<GoalHandle> & goal_handle, std::size_t index,
                const Program & program, const Point & point, bool & cancelled,
                std::string & err)
  {
    const Step & step = program.steps[index];
    publishFeedback(goal_handle, index, program.steps.size(), &step,
                    ExecuteProgram::Feedback::STATUS_PLANNING);

    move_group_->setStartStateToCurrentState();
    move_group_->setPoseReferenceFrame(point.frame);   // loader enforces base_link
    const double ov = override_;
    move_group_->setMaxVelocityScalingFactor(step.velocity * ov);
    move_group_->setMaxAccelerationScalingFactor(step.acceleration * ov);

    moveit_msgs::msg::RobotTrajectory traj_msg;
    const std::vector<geometry_msgs::msg::Pose> waypoints{point.pose};
    const double fraction =
      move_group_->computeCartesianPath(waypoints, cartesian_eef_step_, traj_msg, true);
    if (goal_handle->is_canceling()) {
      cancelled = true;
      return true;
    }
    if (fraction < 0.999) {
      err = "linear path to '" + step.target + "' is only " +
            std::to_string(static_cast<int>(fraction * 100.0)) +
            "% feasible (collision, joint limit or IK failure on the line — "
            "note: KDL cannot follow lines through the joint_5 = 0 wrist singularity)";
      return false;
    }
    if (traj_msg.joint_trajectory.points.size() < 2) {
      return true;   // already at the target pose — nothing to execute
    }

    publishFeedback(goal_handle, index, program.steps.size(), &step,
                    ExecuteProgram::Feedback::STATUS_MOVING);
    const auto code = move_group_->execute(traj_msg);
    if (goal_handle->is_canceling()) {
      cancelled = true;
      return true;
    }
    if (code != moveit::core::MoveItErrorCode::SUCCESS) {
      err = "execution of linear move to '" + step.target + "' failed (MoveIt error " +
            std::to_string(code.val) + ")";
      return false;
    }
    return true;
  }

  // KRL/Pilz move (phase 7): routes a single step through the Pilz Industrial
  // Motion Planner. ptp -> PTP (joint or pose target), lin -> LIN (pose only,
  // enforced by crossValidate). Sequential for now — c_dis blending requires the
  // MoveGroupSequence mode (next sub-step) and is logged + ignored here. The
  // pipeline is reset to OMPL afterwards so legacy move_j/move_l keep their
  // validated planners.
  bool runPilz(const std::shared_ptr<GoalHandle> & goal_handle, std::size_t index,
               const Program & program, const Point & point, bool & cancelled,
               std::string & err)
  {
    const Step & step = program.steps[index];
    publishFeedback(goal_handle, index, program.steps.size(), &step,
                    ExecuteProgram::Feedback::STATUS_PLANNING);

    if (step.c_dis > 0.0) {
      RCLCPP_WARN(get_logger(),
        "step %zu (%s): c_dis=%.3f m ignored — blending needs the Pilz sequence "
        "mode (not yet implemented); stopping at the point",
        index + 1, step.type.c_str(), step.c_dis);
    }

    move_group_->setStartStateToCurrentState();
    move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group_->setPlannerId(step.type == "lin" ? "LIN" : "PTP");
    const double ov = override_;
    move_group_->setMaxVelocityScalingFactor(step.velocity * ov);
    move_group_->setMaxAccelerationScalingFactor(step.acceleration * ov);

    if (point.type == Point::Type::kJoint) {
      std::map<std::string, double> target;
      for (std::size_t i = 0; i < joint_names_.size(); ++i) {
        target[joint_names_[i]] = point.joints[i];
      }
      move_group_->setJointValueTarget(target);
    } else {
      move_group_->setPoseReferenceFrame(point.frame);   // loader enforces base_link
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = point.frame;
      ps.pose = point.pose;
      move_group_->setPoseTarget(ps);
    }

    MoveGroupInterface::Plan plan;
    auto code = move_group_->plan(plan);
    move_group_->clearPoseTargets();
    move_group_->setPlanningPipelineId("ompl");   // restore default for legacy steps
    if (goal_handle->is_canceling()) {
      cancelled = true;
      return true;
    }
    if (code != moveit::core::MoveItErrorCode::SUCCESS) {
      err = step.type + " planning to '" + step.target + "' failed (Pilz error " +
            std::to_string(code.val) + ")";
      if (step.type == "lin") {
        err += " — note: LIN cannot pass the joint_5 = 0 wrist singularity";
      }
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
      err = step.type + " execution of '" + step.target + "' failed (MoveIt error " +
            std::to_string(code.val) + ")";
      return false;
    }
    return true;
  }

  // Pilz "blend through" (phase 7): run the consecutive move steps [i0, i1) as
  // ONE MotionSequenceRequest via /sequence_move_group. Each step becomes a
  // MotionSequenceItem (planner per type; the last item's blend radius is forced
  // to 0 as Pilz requires). circ adds its via point as an "interim" path
  // constraint. Only the first item carries a start state (the live robot state);
  // the rest stay empty so Pilz chains each from the previous.
  //   apply_blend: use each step's c_dis as the blend radius (else stop at every point).
  //   dry_run:     plan only and animate the RViz ghost — never execute (phase 8).
  bool runBlendedRun(const std::shared_ptr<GoalHandle> & goal_handle, std::size_t i0,
                     std::size_t i1, const Program & program, const PointMap & points,
                     bool & cancelled, std::string & err, bool dry_run, bool apply_blend)
  {
    publishFeedback(goal_handle, i0, program.steps.size(), &program.steps[i0],
                    ExecuteProgram::Feedback::STATUS_PLANNING);

    if (!seq_client_->wait_for_action_server(2s)) {
      err = "/sequence_move_group unavailable — is the Pilz MoveGroupSequenceAction "
            "capability loaded in move_group? (moveit.launch.py)";
      return false;
    }

    const auto * jmg = move_group_->getRobotModel()->getJointModelGroup(planning_group_);
    const double ov = override_;

    moveit_msgs::msg::MotionSequenceRequest seq;
    bool has_circ = false;
    for (std::size_t k = i0; k < i1; ++k) {
      const Step & s = program.steps[k];
      const Point & pt = points.at(s.target);
      if (s.type == "circ") has_circ = true;

      moveit_msgs::msg::MotionSequenceItem item;
      auto & req = item.req;
      req.group_name = planning_group_;
      req.pipeline_id = "pilz_industrial_motion_planner";
      req.planner_id = pilzPlannerId(s.type);
      req.num_planning_attempts = 1;
      req.allowed_planning_time = 5.0;
      req.max_velocity_scaling_factor = std::clamp(s.velocity * ov, 0.0, 1.0);
      req.max_acceleration_scaling_factor = std::clamp(s.acceleration * ov, 0.0, 1.0);

      if (pt.type == Point::Type::kJoint) {
        moveit::core::RobotState gs(move_group_->getRobotModel());
        gs.setToDefaultValues();
        gs.setJointGroupPositions(jmg, pt.joints);
        gs.update();
        req.goal_constraints.push_back(
          kinematic_constraints::constructGoalConstraints(gs, jmg, 1e-4));
      } else {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = pt.frame;   // loader enforces base_link
        ps.pose = pt.pose;
        req.goal_constraints.push_back(
          kinematic_constraints::constructGoalConstraints(pose_reference_link_, ps, 1e-3, 1e-2));
      }

      if (s.type == "circ") {
        // Pilz reads the via point from a position constraint on the EE link in
        // path_constraints; the constraint name selects interim vs centre.
        const Point & via = points.at(s.via);
        moveit_msgs::msg::PositionConstraint pc;
        pc.header.frame_id = via.frame;
        pc.link_name = pose_reference_link_;
        pc.constraint_region.primitive_poses.resize(1);
        pc.constraint_region.primitive_poses[0].position = via.pose.position;
        pc.constraint_region.primitive_poses[0].orientation.w = 1.0;
        pc.weight = 1.0;
        req.path_constraints.name = "interim";
        req.path_constraints.position_constraints.push_back(pc);
      }

      // Pilz requires the last segment's blend radius to be 0 (stop on arrival);
      // without blend mode every segment stops too (radius 0).
      item.blend_radius = (apply_blend && (k + 1 != i1)) ? s.c_dis : 0.0;
      seq.items.push_back(std::move(item));
    }

    // Give the first segment a populated start state (the live robot state) so
    // move_group's start-state adapters don't log "Found empty JointState
    // message". move_group executes from the current state regardless; this only
    // quiets that noise. Subsequent items stay empty so Pilz chains them.
    if (!seq.items.empty()) {
      if (auto cur = move_group_->getCurrentState(1.0)) {
        moveit::core::robotStateToRobotStateMsg(*cur, seq.items.front().req.start_state);
      }
    }

    MoveGroupSequence::Goal goal;
    goal.request = seq;
    goal.planning_options.plan_only = dry_run;   // dry run: plan + visualize, never execute

    auto send_future = seq_client_->async_send_goal(goal);
    if (send_future.wait_for(15s) != std::future_status::ready) {
      err = "sequence goal send timed out";
      return false;
    }
    auto gh = send_future.get();
    if (!gh) {
      err = "blended sequence rejected by move_group (check blend radii / singularities)";
      return false;
    }
    { std::lock_guard<std::mutex> lock(seq_mtx_); seq_goal_ = gh; }

    publishFeedback(goal_handle, i0, program.steps.size(), &program.steps[i0],
                    ExecuteProgram::Feedback::STATUS_MOVING);
    auto result_future = seq_client_->async_get_result(gh);
    while (result_future.wait_for(100ms) != std::future_status::ready) {
      if (!rclcpp::ok()) {
        err = "shutdown during blended sequence";
        std::lock_guard<std::mutex> lock(seq_mtx_); seq_goal_.reset();
        return false;
      }
    }
    const auto wrapped = result_future.get();
    { std::lock_guard<std::mutex> lock(seq_mtx_); seq_goal_.reset(); }

    if (goal_handle->is_canceling() ||
        wrapped.code == rclcpp_action::ResultCode::CANCELED) {
      cancelled = true;
      return true;
    }
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
      const int ec = wrapped.result ? wrapped.result->response.error_code.val : 0;
      err = "blended sequence (steps " + std::to_string(i0 + 1) + "-" +
            std::to_string(i1) + ") failed (MoveIt error " + std::to_string(ec) +
            " — check blend radii, reachability or the joint_5 = 0 singularity)";
      if (has_circ) {
        err += ". For circ: start, via and target must not be collinear and must "
               "define a valid arc plane (Pilz: \"Plane for motion is not properly "
               "defined\")";
      }
      return false;
    }
    if (dry_run && wrapped.result) {
      animatePreview(wrapped.result->response, goal_handle, cancelled);
    }
    return true;
  }

  // Dry-run preview: publish the planned trajectories as the RViz ghost
  // (/display_planned_path) and block for their total duration so the animation
  // plays before the next sequence overwrites it. Cancel-responsive.
  void animatePreview(const moveit_msgs::msg::MotionSequenceResponse & resp,
                      const std::shared_ptr<GoalHandle> & goal_handle, bool & cancelled)
  {
    if (resp.planned_trajectories.empty()) return;
    moveit_msgs::msg::DisplayTrajectory disp;
    disp.model_id = move_group_->getRobotModel()->getName();
    disp.trajectory_start = resp.sequence_start;
    disp.trajectory = resp.planned_trajectories;
    display_pub_->publish(disp);

    double secs = 0.0;
    for (const auto & t : resp.planned_trajectories) {
      if (!t.joint_trajectory.points.empty()) {
        const auto & d = t.joint_trajectory.points.back().time_from_start;
        secs += static_cast<double>(d.sec) + static_cast<double>(d.nanosec) * 1e-9;
      }
    }
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(secs);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < end) {
      if (goal_handle->is_canceling()) { cancelled = true; return; }
      std::this_thread::sleep_for(50ms);
    }
  }

  // --------------------------------------------------------------------------
  // Point services (/teach_point, /list_points, /delete_point)
  // --------------------------------------------------------------------------
  void handleTeach(const std::shared_ptr<TeachPoint::Request> & req,
                   std::shared_ptr<TeachPoint::Response> & resp)
  {
    resp->success = false;
    if (!isValidPointName(req->name)) {
      resp->message = "invalid point name '" + req->name +
                      "' (allowed: ^[A-Za-z_][A-Za-z0-9_]*$)";
      return;
    }
    if (req->type != TeachPoint::Request::TYPE_JOINT &&
        req->type != TeachPoint::Request::TYPE_POSE) {
      resp->message = "unknown point type";
      return;
    }
    // Teach only from a defined resting state (servo holding or jogging).
    if (robot_state_ != RobotStateMsg::HOLD && robot_state_ != RobotStateMsg::JOG) {
      resp->message = "teaching requires state HOLD or JOG";
      return;
    }

    PointMap points;
    try {
      if (std::filesystem::exists(points_file_)) points = loadPoints(points_file_);
    } catch (const std::exception & e) {
      resp->message = std::string("cannot load point database: ") + e.what();
      return;
    }
    if (points.count(req->name) && !req->overwrite) {
      resp->message = "point '" + req->name + "' already exists (set overwrite to replace)";
      return;
    }

    Point p;
    if (req->type == TeachPoint::Request::TYPE_JOINT) {
      p.type = Point::Type::kJoint;
      for (const char * joint : kArmJoints) {
        const auto it = joint_pos_.find(joint);
        if (it == joint_pos_.end()) {
          resp->message = std::string("no /joint_states value for ") + joint + " yet";
          return;
        }
        p.joints.push_back(it->second);
      }
    } else {
      p.type = Point::Type::kPose;
      p.frame = "base_link";
      geometry_msgs::msg::TransformStamped tf;
      try {
        tf = tf_buffer_->lookupTransform("base_link", pose_reference_link_,
                                         tf2::TimePointZero, 500ms);
      } catch (const std::exception & e) {
        resp->message = std::string("TF base_link -> ") + pose_reference_link_ +
                        " unavailable: " + e.what();
        return;
      }
      p.pose.position.x = tf.transform.translation.x;
      p.pose.position.y = tf.transform.translation.y;
      p.pose.position.z = tf.transform.translation.z;
      p.pose.orientation = tf.transform.rotation;
    }

    points[req->name] = std::move(p);
    try {
      savePoints(points_file_, points);
    } catch (const std::exception & e) {
      resp->message = std::string("cannot write point database: ") + e.what();
      return;
    }
    resp->success = true;
    resp->message = "taught '" + req->name + "' (" +
                    (req->type == TeachPoint::Request::TYPE_JOINT ? "joint" : "pose") + ")";
    RCLCPP_INFO(get_logger(), "%s", resp->message.c_str());
    publishPointMarkers();
  }

  void handleList(std::shared_ptr<ListPoints::Response> & resp)
  {
    resp->success = false;
    if (!std::filesystem::exists(points_file_)) {
      resp->message = "point database not found: " + points_file_;
      return;
    }
    PointMap points;
    try {
      points = loadPoints(points_file_);   // re-read: VS Code edits show up live
    } catch (const std::exception & e) {
      resp->message = e.what();
      return;
    }
    for (const auto & [name, p] : points) {   // std::map -> sorted by name
      resp->names.push_back(name);
      resp->types.push_back(p.type == Point::Type::kJoint
                              ? TeachPoint::Request::TYPE_JOINT
                              : TeachPoint::Request::TYPE_POSE);
    }
    resp->success = true;
    publishPointMarkers();   // panel refresh doubles as marker refresh
  }

  void handleDelete(const std::shared_ptr<DeletePoint::Request> & req,
                    std::shared_ptr<DeletePoint::Response> & resp)
  {
    resp->success = false;
    PointMap points;
    try {
      if (std::filesystem::exists(points_file_)) points = loadPoints(points_file_);
    } catch (const std::exception & e) {
      resp->message = std::string("cannot load point database: ") + e.what();
      return;
    }
    if (points.erase(req->name) == 0) {
      resp->message = "point '" + req->name + "' does not exist";
      return;
    }
    try {
      savePoints(points_file_, points);
    } catch (const std::exception & e) {
      resp->message = std::string("cannot write point database: ") + e.what();
      return;
    }
    resp->success = true;
    resp->message = "deleted '" + req->name + "'";
    RCLCPP_INFO(get_logger(), "%s", resp->message.c_str());
    publishPointMarkers();
  }

  // --------------------------------------------------------------------------
  // Helpers
  // --------------------------------------------------------------------------
  bool ensureMoveGroup(std::string & err)
  {
    if (move_group_) return true;
    std::shared_ptr<MoveGroupInterface> mg;
    try {
      mg = std::make_shared<MoveGroupInterface>(
        shared_from_this(), planning_group_, std::shared_ptr<tf2_ros::Buffer>(),
        rclcpp::Duration::from_seconds(15.0));
    } catch (const std::exception & e) {
      err = e.what();
      return false;
    }
    joint_names_ = mg->getJointNames();
    if (joint_names_.size() != 6) {
      err = "planning group '" + planning_group_ + "' has " +
            std::to_string(joint_names_.size()) + " active joints, expected 6";
      return false;
    }
    {
      // Guarded: service callbacks (cancel, marker FK) read move_group_ from
      // the main spin while this runs on the worker thread.
      std::lock_guard<std::mutex> lock(move_group_mtx_);
      move_group_ = mg;
    }
    publishPointMarkers();   // joint-point FK is possible from now on
    return true;
  }

  std::shared_ptr<MoveGroupInterface> moveGroup()
  {
    std::lock_guard<std::mutex> lock(move_group_mtx_);
    return move_group_;
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

  void publishOverride()
  {
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(override_.load());
    override_pub_->publish(msg);
  }

  // Republished after every teach/delete, on /list_points (panel refresh) and
  // when the MoveGroup becomes available (joint-point FK needs the model).
  void publishPointMarkers()
  {
    PointMap points;
    try {
      if (std::filesystem::exists(points_file_)) points = loadPoints(points_file_);
    } catch (const std::exception &) {
      return;  // invalid file: keep the last published markers
    }

    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);

    int id = 0;
    for (const auto & [name, p] : points) {
      geometry_msgs::msg::Pose pose;
      std::string frame;
      bool is_pose = (p.type == Point::Type::kPose);
      if (is_pose) {
        pose = p.pose;
        frame = p.frame;
      } else {
        auto mg = moveGroup();
        if (!mg) continue;  // FK possible only once MoveIt was contacted
        moveit::core::RobotState state(mg->getRobotModel());
        state.setToDefaultValues();
        state.setJointGroupPositions(planning_group_, p.joints);
        state.update();
        const Eigen::Isometry3d & tf = state.getGlobalLinkTransform(pose_reference_link_);
        const Eigen::Quaterniond q(tf.rotation());
        pose.position.x = tf.translation().x();
        pose.position.y = tf.translation().y();
        pose.position.z = tf.translation().z();
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        frame = mg->getRobotModel()->getModelFrame();
      }

      visualization_msgs::msg::Marker sphere;
      sphere.header.frame_id = frame;
      sphere.ns = "points";
      sphere.id = id++;
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.action = visualization_msgs::msg::Marker::ADD;
      sphere.pose = pose;
      sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.012;
      sphere.color.a = 0.9f;
      if (is_pose) {  // pose = orange, joint = blue
        sphere.color.r = 1.0f; sphere.color.g = 0.55f; sphere.color.b = 0.0f;
      } else {
        sphere.color.r = 0.15f; sphere.color.g = 0.45f; sphere.color.b = 1.0f;
      }
      arr.markers.push_back(sphere);

      visualization_msgs::msg::Marker text = sphere;
      text.ns = "labels";
      text.id = id++;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.text = name;
      text.pose.position.z += 0.025;
      text.scale.x = text.scale.y = 0.0;
      text.scale.z = 0.018;  // text height (m)
      text.color.r = text.color.g = text.color.b = 1.0f;
      arr.markers.push_back(text);
    }
    marker_pub_->publish(arr);
  }

  // --- Parameters ---
  std::string programs_dir_;
  std::string points_file_;
  std::string planning_group_;
  std::string pose_reference_link_;
  double cartesian_eef_step_{0.005};

  // --- ROS ---
  rclcpp_action::Server<ExecuteProgram>::SharedPtr server_;
  rclcpp_action::Client<MoveGroupSequence>::SharedPtr seq_client_;  // Pilz blend mode
  std::shared_ptr<rclcpp_action::ClientGoalHandle<MoveGroupSequence>> seq_goal_;  // active, for cancel
  std::mutex seq_mtx_;
  rclcpp::Client<SetRobotState>::SharedPtr state_client_;
  rclcpp::Subscription<RobotStateMsg>::SharedPtr state_sub_;
  rclcpp::Service<TeachPoint>::SharedPtr teach_srv_;
  rclcpp::Service<ListPoints>::SharedPtr list_srv_;
  rclcpp::Service<DeletePoint>::SharedPtr delete_srv_;
  rclcpp::Service<Trigger>::SharedPtr pause_srv_;
  rclcpp::Service<Trigger>::SharedPtr resume_srv_;
  rclcpp::Service<SetProgramOverride>::SharedPtr override_srv_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr override_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_pub_;  // dry-run ghost

  // --- Teaching sources (filled and read on the main spin only) ---
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  std::map<std::string, double> joint_pos_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // --- Execution ---
  std::shared_ptr<MoveGroupInterface> move_group_;  // lazy (move_group up later)
  std::mutex move_group_mtx_;                       // guards move_group_ assignment/reads
  std::vector<std::string> joint_names_;
  std::thread worker_;
  std::atomic<bool> busy_{false};
  std::atomic<bool> pause_requested_{false};
  std::atomic<double> override_{1.0};
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
