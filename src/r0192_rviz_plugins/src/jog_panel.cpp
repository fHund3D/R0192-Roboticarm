#include "r0192_rviz_plugins/jog_panel.hpp"

#include <rviz_common/display_context.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/properties/property.hpp>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QFrame>
#include <QFont>

#include <cmath>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/time.h>

namespace r0192_rviz_plugins
{

namespace
{
// Servo ROS API (node name "servo_node").
constexpr const char * kJointTopic   = "/servo_node/delta_joint_cmds";
constexpr const char * kTwistTopic   = "/servo_node/delta_twist_cmds";
constexpr const char * kSwitchTypeSrv = "/servo_node/switch_command_type";

// TF frames for Cartesian jogging.
constexpr const char * kBaseFrame = "base_link";
// Dedicated TCP frame (tool0 convention): +Z = gripper approach direction.
constexpr const char * kToolFrame = "tcp";

// Scales the position readout from model units to the real arm. The URDF is now
// 1:1 (mesh scale 0.001 mapping STL-mm to metres), so this is 1.0. Kept as a
// named constant in case a future scale mismatch needs correcting.
constexpr double kModelToRealScale = 1.0;

constexpr int kPublishPeriodMs = 20;   // 50 Hz command stream while held

const char * jointName(int dof)
{
  static const char * names[6] = {
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
  return names[dof];
}
}  // namespace

JogPanel::JogPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * root = new QVBoxLayout;

  // --- Emergency stop (always at the top, always available) ---
  estop_btn_ = new QPushButton("EMERGENCY STOP");
  estop_btn_->setStyleSheet(
    "QPushButton { background-color: #c62828; color: white; font-weight: bold; "
    "font-size: 16px; padding: 10px; border-radius: 4px; }"
    "QPushButton:pressed { background-color: #8e0000; }");
  estop_btn_->setMinimumHeight(48);
  root->addWidget(estop_btn_);

  // --- Reset (clears latched driver faults after an e-stop) ---
  reset_btn_ = new QPushButton("Reset (clear faults)");
  reset_btn_->setToolTip("Clear latched driver faults after an emergency stop, "
                         "then re-enable the servos.");
  root->addWidget(reset_btn_);

  // --- Mode selector ---
  auto * mode_box = new QGroupBox("Mode");
  auto * mode_layout = new QHBoxLayout;
  mode_joint_ = new QRadioButton("Joints");
  mode_base_  = new QRadioButton("Cartesian (Base)");
  mode_tool_  = new QRadioButton("Tool");
  mode_joint_->setChecked(true);
  mode_layout->addWidget(mode_joint_);
  mode_layout->addWidget(mode_base_);
  mode_layout->addWidget(mode_tool_);
  mode_box->setLayout(mode_layout);
  root->addWidget(mode_box);

  // --- 6 jog rows: [−][+] on the left, name in the middle, live value right ---
  auto * grid = new QGridLayout;
  for (int dof = 0; dof < 6; ++dof) {
    minus_btn_[dof] = new QPushButton("−");
    plus_btn_[dof]  = new QPushButton("+");
    dof_label_[dof] = new QLabel;
    dof_label_[dof]->setAlignment(Qt::AlignCenter);
    QFont f = dof_label_[dof]->font();
    f.setBold(true);
    dof_label_[dof]->setFont(f);
    value_label_[dof] = new QLabel("—");
    value_label_[dof]->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    value_label_[dof]->setStyleSheet("font-family: monospace;");
    minus_btn_[dof]->setMinimumWidth(40);
    plus_btn_[dof]->setMinimumWidth(40);
    grid->addWidget(minus_btn_[dof],  dof, 0);
    grid->addWidget(plus_btn_[dof],   dof, 1);
    grid->addWidget(dof_label_[dof],  dof, 2);
    grid->addWidget(value_label_[dof], dof, 3);
  }
  grid->setColumnStretch(3, 1);   // live-value column takes the extra width
  root->addLayout(grid);

  // --- Speed slider ---
  auto * speed_layout = new QHBoxLayout;
  speed_layout->addWidget(new QLabel("Speed"));
  speed_slider_ = new QSlider(Qt::Horizontal);
  speed_slider_->setRange(1, 100);
  speed_slider_->setValue(25);
  speed_value_ = new QLabel("25 %");
  speed_value_->setMinimumWidth(44);
  speed_layout->addWidget(speed_slider_);
  speed_layout->addWidget(speed_value_);
  root->addLayout(speed_layout);

  // --- Master jog-active toggle ---
  jog_active_btn_ = new QPushButton("Enable Jog");
  jog_active_btn_->setCheckable(true);
  jog_active_btn_->setChecked(false);
  root->addWidget(jog_active_btn_);

  // --- Operator controls (merged from the former OperatorPanel) ---
  auto * sep = new QFrame;
  sep->setFrameShape(QFrame::HLine);
  sep->setFrameShadow(QFrame::Sunken);
  root->addWidget(sep);

  // Servos master toggle (DISABLED <-> HOLD). Hardware starts de-energized.
  enable_btn_ = new QPushButton;
  enable_btn_->setCheckable(true);
  enable_btn_->setChecked(false);
  root->addWidget(enable_btn_);

  // MoveIt toggle (HOLD <-> MOVEIT): only when active may move_group execute.
  moveit_btn_ = new QPushButton;
  moveit_btn_->setCheckable(true);
  moveit_btn_->setChecked(false);
  root->addWidget(moveit_btn_);

  homing_btn_ = new QPushButton("Homing");
  root->addWidget(homing_btn_);

  // Goal-marker visibility: hide MoveIt's interactive goal state while jogging
  // so the orange query robot doesn't obscure the live arm pose.
  query_goal_btn_ = new QPushButton;
  query_goal_btn_->setCheckable(true);
  query_goal_btn_->setChecked(true);   // goal shown by default (matches rviz cfg)
  query_goal_btn_->setText("Goal Marker: shown");
  root->addWidget(query_goal_btn_);

  status_label_ = new QLabel("Waiting for robot state …");
  status_label_->setWordWrap(true);
  root->addWidget(status_label_);
  root->addStretch();
  setLayout(root);

  updateDofLabels();
  updateUiForState();   // reflect the initial (DISABLED) state

  // --- Connections ---
  connect(mode_joint_, &QRadioButton::toggled, this, &JogPanel::onModeChanged);
  connect(mode_base_,  &QRadioButton::toggled, this, &JogPanel::onModeChanged);
  connect(mode_tool_,  &QRadioButton::toggled, this, &JogPanel::onModeChanged);

  for (int dof = 0; dof < 6; ++dof) {
    connect(minus_btn_[dof], &QPushButton::pressed,  this, [this, dof]() { onAxisPressed(dof, -1.0); });
    connect(plus_btn_[dof],  &QPushButton::pressed,  this, [this, dof]() { onAxisPressed(dof, +1.0); });
    connect(minus_btn_[dof], &QPushButton::released, this, &JogPanel::onAxisReleased);
    connect(plus_btn_[dof],  &QPushButton::released, this, &JogPanel::onAxisReleased);
  }

  connect(speed_slider_, &QSlider::valueChanged, this,
          [this](int v) { speed_value_->setText(QString::number(v) + " %"); });
  connect(estop_btn_, &QPushButton::clicked, this, &JogPanel::onEStopClicked);
  connect(reset_btn_, &QPushButton::clicked, this, &JogPanel::onResetClicked);
  connect(jog_active_btn_, &QPushButton::toggled, this, &JogPanel::onJogActiveToggled);
  connect(moveit_btn_, &QPushButton::toggled, this, &JogPanel::onMoveItToggled);
  connect(homing_btn_, &QPushButton::clicked, this, &JogPanel::onHomingClicked);
  connect(enable_btn_, &QPushButton::toggled, this, &JogPanel::onEnableToggled);
  connect(query_goal_btn_, &QPushButton::toggled, this, &JogPanel::onQueryGoalToggled);
}

void JogPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  joint_pub_ = node_->create_publisher<control_msgs::msg::JointJog>(kJointTopic, rclcpp::QoS(10));
  twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(kTwistTopic, rclcpp::QoS(10));
  switch_type_client_ = node_->create_client<moveit_msgs::srv::ServoCommandType>(kSwitchTypeSrv);

  // Central state machine: request transitions + mirror the authoritative state.
  set_state_client_ = node_->create_client<r0192_interfaces::srv::SetRobotState>("/set_robot_state");
  estop_client_ = node_->create_client<std_srvs::srv::Trigger>("/e_stop");
  reset_client_ = node_->create_client<std_srvs::srv::Trigger>("/robot_reset");
  state_sub_ = node_->create_subscription<r0192_interfaces::msg::RobotState>(
    "/robot_state", rclcpp::QoS(1).transient_local(),
    [this](const r0192_interfaces::msg::RobotState::ConstSharedPtr msg) {
      onRobotState(msg->state, QString::fromStdString(msg->status));
    });

  // Live readout sources: joint angles from /joint_states, TCP pose from TF.
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::QoS(10),
    [this](const sensor_msgs::msg::JointState::ConstSharedPtr msg) {
      const size_t n = std::min(msg->name.size(), msg->position.size());
      for (size_t i = 0; i < n; ++i) {
        joint_pos_[msg->name[i]] = msg->position[i];
      }
    });
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_);

  // Timer drives the press-and-hold command stream; only runs while jog active.
  pub_timer_ = new QTimer(this);
  pub_timer_->setInterval(kPublishPeriodMs);
  connect(pub_timer_, &QTimer::timeout, this, &JogPanel::onPublishTick);

  // Timer refreshes the live readout column (always running).
  value_timer_ = new QTimer(this);
  value_timer_->setInterval(100);   // 10 Hz
  connect(value_timer_, &QTimer::timeout, this, &JogPanel::onValueTick);
  value_timer_->start();
}

void JogPanel::onModeChanged()
{
  Mode m = Mode::Joint;
  if (mode_base_->isChecked()) {
    m = Mode::CartesianBase;
  } else if (mode_tool_->isChecked()) {
    m = Mode::Tool;
  }
  setMode(m);
}

void JogPanel::setMode(Mode mode)
{
  if (mode == mode_) {
    return;
  }
  mode_ = mode;
  updateDofLabels();

  // Stop any in-flight jog when switching modes.
  active_dof_ = -1;
  active_sign_ = 0.0;

  // If jogging is active, retarget Servo to the new command type.
  if (robot_state_ == r0192_interfaces::msg::RobotState::JOG) {
    applyCommandType();
  }
}

void JogPanel::updateDofLabels()
{
  static const char * cart[6] = {"X", "Y", "Z", "RX", "RY", "RZ"};
  for (int dof = 0; dof < 6; ++dof) {
    if (mode_ == Mode::Joint) {
      dof_label_[dof]->setText(QString("Joint %1").arg(dof + 1));
    } else {
      dof_label_[dof]->setText(cart[dof]);
    }
  }
}

void JogPanel::onJogActiveToggled(bool checked)
{
  using RS = r0192_interfaces::msg::RobotState;
  // Request the transition; the UI (and Servo command stream) follow the
  // confirmed /robot_state, so do nothing else here.
  requestState(checked ? RS::JOG : RS::HOLD);
}

void JogPanel::onAxisPressed(int dof, double sign)
{
  if (robot_state_ != r0192_interfaces::msg::RobotState::JOG) {
    return;  // buttons are disabled outside JOG, but guard anyway
  }
  active_dof_ = dof;
  active_sign_ = sign;
  onPublishTick();  // publish immediately for snappy response
}

void JogPanel::onAxisReleased()
{
  active_dof_ = -1;
  active_sign_ = 0.0;
  // Send one zero command so Servo halts immediately instead of after timeout.
  if (mode_ == Mode::Joint) {
    control_msgs::msg::JointJog msg;
    msg.header.stamp = node_->now();
    for (int d = 0; d < 6; ++d) {
      msg.joint_names.push_back(jointName(d));
      msg.velocities.push_back(0.0);
    }
    joint_pub_->publish(msg);
  } else {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = (mode_ == Mode::Tool) ? kToolFrame : kBaseFrame;
    twist_pub_->publish(msg);
  }
}

void JogPanel::onPublishTick()
{
  if (active_dof_ < 0) {
    return;
  }
  const double value = active_sign_ * speedFactor();

  if (mode_ == Mode::Joint) {
    control_msgs::msg::JointJog msg;
    msg.header.stamp = node_->now();
    msg.joint_names.push_back(jointName(active_dof_));
    msg.velocities.push_back(value);
    joint_pub_->publish(msg);
  } else {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = (mode_ == Mode::Tool) ? kToolFrame : kBaseFrame;
    switch (active_dof_) {
      case 0: msg.twist.linear.x  = value; break;
      case 1: msg.twist.linear.y  = value; break;
      case 2: msg.twist.linear.z  = value; break;
      case 3: msg.twist.angular.x = value; break;
      case 4: msg.twist.angular.y = value; break;
      case 5: msg.twist.angular.z = value; break;
      default: break;
    }
    twist_pub_->publish(msg);
  }
}

void JogPanel::onValueTick()
{
  if (mode_ == Mode::Joint) {
    // Live joint angles from /joint_states, in degrees.
    for (int dof = 0; dof < 6; ++dof) {
      auto it = joint_pos_.find(jointName(dof));
      if (it == joint_pos_.end()) {
        value_label_[dof]->setText("—");
      } else {
        value_label_[dof]->setText(
          QString::asprintf("%+.3f°", it->second * 180.0 / M_PI));
      }
    }
    return;
  }

  // Cartesian / Tool: live TCP pose in the base frame (position [mm] + RPY [°]).
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(kBaseFrame, kToolFrame, tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    for (int dof = 0; dof < 6; ++dof) {
      value_label_[dof]->setText("—");
    }
    return;
  }
  const auto & t = tf.transform.translation;
  tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y,
                    tf.transform.rotation.z, tf.transform.rotation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  const double r2d = 180.0 / M_PI;
  const double m2mm = 1000.0 * kModelToRealScale;   // model metres -> real mm
  value_label_[0]->setText(QString::asprintf("%+.1f mm", t.x * m2mm));
  value_label_[1]->setText(QString::asprintf("%+.1f mm", t.y * m2mm));
  value_label_[2]->setText(QString::asprintf("%+.1f mm", t.z * m2mm));
  value_label_[3]->setText(QString::asprintf("%+.1f°", roll  * r2d));
  value_label_[4]->setText(QString::asprintf("%+.1f°", pitch * r2d));
  value_label_[5]->setText(QString::asprintf("%+.1f°", yaw   * r2d));
}

void JogPanel::applyCommandType()
{
  if (!switch_type_client_->service_is_ready()) {
    setStatus("Servo switch_command_type unavailable.", false);
    return;
  }
  auto req = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
  req->command_type = (mode_ == Mode::Joint)
    ? moveit_msgs::srv::ServoCommandType::Request::JOINT_JOG
    : moveit_msgs::srv::ServoCommandType::Request::TWIST;
  switch_type_client_->async_send_request(req);
}

void JogPanel::setJogButtonsEnabled(bool enabled)
{
  for (int dof = 0; dof < 6; ++dof) {
    minus_btn_[dof]->setEnabled(enabled);
    plus_btn_[dof]->setEnabled(enabled);
  }
}

double JogPanel::speedFactor() const
{
  return static_cast<double>(speed_slider_->value()) / 100.0;
}

void JogPanel::setStatus(const QString & text, bool ok)
{
  status_label_->setText(text);
  status_label_->setStyleSheet(ok ? "color: #2e7d32;" : "color: #c62828;");
}

// ----------------------------------------------------------------------------
// State-machine client (/set_robot_state + /robot_state)
// ----------------------------------------------------------------------------
void JogPanel::requestState(uint8_t target)
{
  if (!set_state_client_->service_is_ready()) {
    setStatus("State manager unavailable (/set_robot_state). Hardware active?", false);
    updateUiForState();   // revert any optimistic button toggle
    return;
  }
  auto req = std::make_shared<r0192_interfaces::srv::SetRobotState::Request>();
  req->requested_state = target;
  set_state_client_->async_send_request(
    req,
    [this](rclcpp::Client<r0192_interfaces::srv::SetRobotState>::SharedFuture future) {
      auto resp = future.get();
      robot_state_ = resp->current_state;
      setStatus(QString::fromStdString(resp->message), resp->success);
      updateUiForState();
    });
}

void JogPanel::onRobotState(uint8_t state, const QString & status)
{
  prev_robot_state_ = robot_state_;
  robot_state_ = state;
  updateUiForState();
  if (!status.isEmpty()) {
    setStatus(status, state != r0192_interfaces::msg::RobotState::DISABLED);
  }

  // After homing finishes (HOMING -> HOLD), reset the displays so MoveIt's start
  // state re-syncs to the freshly homed pose (same effect as RViz's "Reset").
  using RS = r0192_interfaces::msg::RobotState;
  if (prev_robot_state_ == RS::HOMING && robot_state_ == RS::HOLD) {
    QTimer::singleShot(500, this, [this]() { resetDisplays(); });
  }
}

void JogPanel::updateUiForState()
{
  using RS = r0192_interfaces::msg::RobotState;
  const bool disabled = (robot_state_ == RS::DISABLED);
  const bool hold     = (robot_state_ == RS::HOLD);
  const bool jog      = (robot_state_ == RS::JOG);
  const bool moveit   = (robot_state_ == RS::MOVEIT);
  const bool homing   = (robot_state_ == RS::HOMING);

  // Servos toggle: checked when energised; clickable only in DISABLED/HOLD.
  enable_btn_->blockSignals(true);
  enable_btn_->setChecked(!disabled);
  enable_btn_->setEnabled(disabled || hold);
  enable_btn_->blockSignals(false);
  if (disabled) {
    enable_btn_->setText("Servos Disabled");
    enable_btn_->setStyleSheet("font-weight: bold; color: #c62828;");
  } else {
    enable_btn_->setText("Servos Enabled");
    enable_btn_->setStyleSheet("font-weight: bold; color: #2e7d32;");
  }

  // MoveIt toggle: only reachable from HOLD; leave back to HOLD.
  moveit_btn_->blockSignals(true);
  moveit_btn_->setChecked(moveit);
  moveit_btn_->setEnabled(hold || moveit);
  moveit_btn_->blockSignals(false);
  moveit_btn_->setText(moveit ? "MoveIt: active" : "MoveIt: off");
  moveit_btn_->setStyleSheet(moveit ? "font-weight: bold; color: #2e7d32;" : "");

  // Jog master toggle: only reachable from HOLD; leave back to HOLD.
  jog_active_btn_->blockSignals(true);
  jog_active_btn_->setChecked(jog);
  jog_active_btn_->setEnabled(hold || jog);
  jog_active_btn_->blockSignals(false);
  jog_active_btn_->setText(jog ? "Stop Jog" : "Enable Jog");

  // Homing: only from HOLD.
  homing_btn_->setEnabled(hold);

  // Reset (clear faults) only matters after an e-stop, i.e. from DISABLED.
  reset_btn_->setEnabled(disabled);

  // Per-axis +/- buttons only live in JOG.
  setJogButtonsEnabled(jog);

  // Drive the Servo command stream from the JOG state. (pub_timer_ is created in
  // onInitialize(); this method also runs once from the constructor before that,
  // so guard against the null timer.)
  if (pub_timer_) {
    if (jog && !pub_timer_->isActive()) {
      applyCommandType();       // select JointJog / Twist for the current mode
      pub_timer_->start();
    } else if (!jog && pub_timer_->isActive()) {
      active_dof_ = -1;
      active_sign_ = 0.0;
      pub_timer_->stop();
    }
  }

  if (homing) {
    setStatus("Homing running …", true);
  }
}

void JogPanel::resetDisplays()
{
  auto * ctx = getDisplayContext();
  if (!ctx) {
    return;
  }
  ctx->getRootDisplayGroup()->reset();
  ctx->queueRender();
}

void JogPanel::onEnableToggled(bool checked)
{
  using RS = r0192_interfaces::msg::RobotState;
  // Servos on = HOLD, off = DISABLED. UI follows the confirmed /robot_state.
  requestState(checked ? RS::HOLD : RS::DISABLED);
}

void JogPanel::onMoveItToggled(bool checked)
{
  using RS = r0192_interfaces::msg::RobotState;
  requestState(checked ? RS::MOVEIT : RS::HOLD);
}

void JogPanel::onHomingClicked()
{
  requestState(r0192_interfaces::msg::RobotState::HOMING);
}

void JogPanel::onEStopClicked()
{
  // E-Stop is special: it goes through the dedicated /e_stop service (not
  // /set_robot_state) so it can force DISABLED from ANY state. The UI then
  // follows the /robot_state update back to DISABLED.
  if (!estop_client_->service_is_ready()) {
    setStatus("E-STOP service /e_stop unavailable — is the state manager up?", false);
    return;
  }
  setStatus("EMERGENCY STOP triggered …", false);
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  estop_client_->async_send_request(
    req,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      auto resp = future.get();
      setStatus(QString::fromStdString(resp->message), false);  // red regardless
    });
}

void JogPanel::onResetClicked()
{
  // Reset clears the latched driver faults left by an e-stop (hardware
  // /robot_clear_faults via the state manager), so DISABLED->HOLD works again.
  if (!reset_client_->service_is_ready()) {
    setStatus("Reset service /robot_reset unavailable — is the state manager up?", false);
    return;
  }
  setStatus("Resetting drivers …", true);
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  reset_client_->async_send_request(
    req,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      auto resp = future.get();
      setStatus(QString::fromStdString(resp->message), resp->success);
    });
}

// ----------------------------------------------------------------------------
// MoveIt query-goal-state visibility toggle
// ----------------------------------------------------------------------------
rviz_common::Display * JogPanel::findMotionPlanningDisplay()
{
  auto * ctx = getDisplayContext();
  if (!ctx) {
    return nullptr;
  }
  auto * group = ctx->getRootDisplayGroup();
  for (int i = 0; i < group->numDisplays(); ++i) {
    auto * d = group->getDisplayAt(i);
    if (d && (d->getClassId() == "moveit_rviz_plugin/MotionPlanning" ||
              d->getName() == "MotionPlanning")) {
      return d;
    }
  }
  return nullptr;
}

void JogPanel::onQueryGoalToggled(bool show)
{
  query_goal_btn_->setText(show ? "Goal Marker: shown" : "Goal Marker: hidden");

  auto * d = findMotionPlanningDisplay();
  if (!d) {
    setStatus("MotionPlanning display not found — cannot toggle goal marker.", false);
    return;
  }
  // MotionPlanning → "Planning Request" group → "Query Goal State" bool property.
  d->subProp("Planning Request")->subProp("Query Goal State")->setValue(show);
  getDisplayContext()->queueRender();
}

}  // namespace r0192_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(r0192_rviz_plugins::JogPanel, rviz_common::Panel)
