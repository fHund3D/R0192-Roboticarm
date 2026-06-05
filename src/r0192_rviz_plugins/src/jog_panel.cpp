#include "r0192_rviz_plugins/jog_panel.hpp"

#include <rviz_common/display_context.hpp>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QFont>

namespace r0192_rviz_plugins
{

namespace
{
// Servo ROS API (node name "servo_node").
constexpr const char * kJointTopic   = "/servo_node/delta_joint_cmds";
constexpr const char * kTwistTopic   = "/servo_node/delta_twist_cmds";
constexpr const char * kSwitchTypeSrv = "/servo_node/switch_command_type";
constexpr const char * kPauseSrv     = "/servo_node/pause_servo";

// TF frames for Cartesian jogging.
constexpr const char * kBaseFrame = "base_link";
// Dedicated TCP frame (tool0 convention): +Z = gripper approach direction.
constexpr const char * kToolFrame = "tcp";

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

  // --- 6 x (- / +) jog rows ---
  auto * grid = new QGridLayout;
  for (int dof = 0; dof < 6; ++dof) {
    minus_btn_[dof] = new QPushButton("−");
    plus_btn_[dof]  = new QPushButton("+");
    dof_label_[dof] = new QLabel;
    dof_label_[dof]->setAlignment(Qt::AlignCenter);
    QFont f = dof_label_[dof]->font();
    f.setBold(true);
    dof_label_[dof]->setFont(f);
    minus_btn_[dof]->setMinimumWidth(48);
    plus_btn_[dof]->setMinimumWidth(48);
    grid->addWidget(minus_btn_[dof], dof, 0);
    grid->addWidget(dof_label_[dof], dof, 1);
    grid->addWidget(plus_btn_[dof],  dof, 2);
  }
  grid->setColumnStretch(1, 1);
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

  status_label_ = new QLabel("Jog inactive — MoveIt has control.");
  status_label_->setWordWrap(true);
  root->addWidget(status_label_);
  root->addStretch();
  setLayout(root);

  updateDofLabels();
  setJogButtonsEnabled(false);

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
  connect(jog_active_btn_, &QPushButton::toggled, this, &JogPanel::onJogActiveToggled);
}

void JogPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  joint_pub_ = node_->create_publisher<control_msgs::msg::JointJog>(kJointTopic, rclcpp::QoS(10));
  twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(kTwistTopic, rclcpp::QoS(10));
  switch_type_client_ = node_->create_client<moveit_msgs::srv::ServoCommandType>(kSwitchTypeSrv);
  pause_client_ = node_->create_client<std_srvs::srv::SetBool>(kPauseSrv);

  // Timer drives the press-and-hold command stream; only runs while jog active.
  pub_timer_ = new QTimer(this);
  pub_timer_->setInterval(kPublishPeriodMs);
  connect(pub_timer_, &QTimer::timeout, this, &JogPanel::onPublishTick);
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
  if (jog_active_btn_->isChecked()) {
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
  if (checked) {
    if (!pause_client_->service_is_ready()) {
      setStatus("Servo unavailable (use_servo:=true? hardware active?).", false);
      jog_active_btn_->blockSignals(true);
      jog_active_btn_->setChecked(false);
      jog_active_btn_->blockSignals(false);
      return;
    }
    callPause(false);          // unpause -> Servo takes over arm_controller
    applyCommandType();        // select JointJog / Twist for current mode
    setJogButtonsEnabled(true);
    pub_timer_->start();
    jog_active_btn_->setText("Stop Jog");
    setStatus("Jog active — motors must be enabled.", true);
  } else {
    active_dof_ = -1;
    active_sign_ = 0.0;
    pub_timer_->stop();
    setJogButtonsEnabled(false);
    callPause(true);           // pause -> hand arm_controller back to MoveIt
    jog_active_btn_->setText("Enable Jog");
    setStatus("Jog inactive — MoveIt has control.", true);
  }
}

void JogPanel::onAxisPressed(int dof, double sign)
{
  if (!jog_active_btn_->isChecked()) {
    return;  // buttons are disabled in this state, but guard anyway
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

void JogPanel::callPause(bool pause)
{
  if (!pause_client_->service_is_ready()) {
    return;
  }
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = pause;
  pause_client_->async_send_request(req);
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

}  // namespace r0192_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(r0192_rviz_plugins::JogPanel, rviz_common::Panel)
