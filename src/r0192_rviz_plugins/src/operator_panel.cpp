#include "r0192_rviz_plugins/operator_panel.hpp"

#include <rviz_common/display_context.hpp>
#include <rviz_common/display_group.hpp>

#include <QVBoxLayout>
#include <QFont>
#include <QTimer>

namespace r0192_rviz_plugins
{

OperatorPanel::OperatorPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  homing_btn_ = new QPushButton("Homing");

  // Single checkable toggle: checked = motors enabled, unchecked = disabled.
  // The hardware starts de-energized (on_activate leaves motors_enabled_=false),
  // so the toggle reflects that until the operator turns the motors on.
  enable_btn_ = new QPushButton;
  enable_btn_->setCheckable(true);
  enable_btn_->setChecked(false);   // hardware starts disabled
  updateEnableButton(false);        // initial label/colour (no signal yet)

  status_label_ = new QLabel("Ready.");
  status_label_->setWordWrap(true);

  auto * layout = new QVBoxLayout;
  layout->addWidget(homing_btn_);
  layout->addWidget(enable_btn_);
  layout->addWidget(status_label_);
  layout->addStretch();
  setLayout(layout);

  connect(homing_btn_, &QPushButton::clicked, this, &OperatorPanel::onHomingClicked);
  connect(enable_btn_, &QPushButton::toggled, this, &OperatorPanel::onEnableToggled);
}

void OperatorPanel::onInitialize()
{
  // RViz owns the ROS node; we hang our service clients off it so they are
  // spun by RViz's executor.
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  homing_client_ = node_->create_client<std_srvs::srv::Trigger>("/homing");
  enable_client_ = node_->create_client<std_srvs::srv::SetBool>("/robot_enable");
}

void OperatorPanel::onHomingClicked()
{
  if (!homing_client_->service_is_ready()) {
    setStatus("Service /homing not available (Hardware active? Axis 1 present?).", false);
    return;
  }

  setStatus("Homing running …", true);
  homing_btn_->setEnabled(false);

  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  homing_client_->async_send_request(
    req,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      auto resp = future.get();
      const std::string prefix = resp->success ? "Homing OK: " : "Homing ERROR: ";
      setStatus(QString::fromStdString(prefix + resp->message), resp->success);
      homing_btn_->setEnabled(true);

      // After a successful homing the arm was re-zeroed (all joints 0). Reset
      // the RViz displays — same effect as the "Reset" button — so MoveIt's
      // MotionPlanning start state re-syncs to the homed pose instead of
      // keeping the stale pre-homing query state. Delayed slightly so the
      // zeroed /joint_states first reach MoveIt's planning-scene monitor.
      if (resp->success) {
        QTimer::singleShot(500, this, [this]() { resetDisplays(); });
      }
    });
}

void OperatorPanel::resetDisplays()
{
  auto * ctx = getDisplayContext();
  if (!ctx) {
    return;
  }
  // Recursively reset every display (incl. moveit MotionPlanning), mirroring
  // what VisualizationManager::resetTime() does for the Reset button.
  ctx->getRootDisplayGroup()->reset();
  ctx->queueRender();
}

void OperatorPanel::onEnableToggled(bool checked)
{
  if (!enable_client_->service_is_ready()) {
    setStatus("Service /robot_enable not available (Hardware active?).", false);
    // Revert the toggle so it doesn't show a state that never took effect.
    enable_btn_->blockSignals(true);
    enable_btn_->setChecked(!checked);
    enable_btn_->blockSignals(false);
    updateEnableButton(!checked);
    return;
  }

  updateEnableButton(checked);
  setStatus(checked ? "Servos Enabled …" : "Servos Disabled …", true);

  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = checked;
  enable_client_->async_send_request(
    req,
    [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
      auto resp = future.get();
      setStatus(QString::fromStdString(resp->message), resp->success);
    });
}

void OperatorPanel::updateEnableButton(bool enabled)
{
  if (enabled) {
    enable_btn_->setText("Servos Enabled");
    enable_btn_->setStyleSheet("font-weight: bold; color: #2e7d32;");
  } else {
    enable_btn_->setText("Servos Disabled");
    enable_btn_->setStyleSheet("font-weight: bold; color: #c62828;");
  }
}

void OperatorPanel::setStatus(const QString & text, bool ok)
{
  status_label_->setText(text);
  status_label_->setStyleSheet(ok ? "color: #2e7d32;" : "color: #c62828;");
}

}  // namespace r0192_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(r0192_rviz_plugins::OperatorPanel, rviz_common::Panel)
