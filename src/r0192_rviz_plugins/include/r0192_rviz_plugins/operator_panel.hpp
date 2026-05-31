#pragma once

#include <rviz_common/panel.hpp>

#include <QPushButton>
#include <QLabel>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace r0192_rviz_plugins
{

// ============================================================================
// OperatorPanel
//
// Small RViz 2 dock panel with operator buttons for the R0192 arm:
//   - "Homing starten"  -> calls /homing        (std_srvs/Trigger)
//   - Enable toggle      -> calls /robot_enable  (std_srvs/SetBool); checked =
//                           motors enabled, unchecked = motors disabled.
// A status label shows the latest service result (green = ok, red = error).
//
// All service calls are asynchronous so the GUI thread never blocks. RViz spins
// its node on the main (GUI) thread, so the response callbacks may touch the Qt
// widgets directly.
// ============================================================================
class OperatorPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit OperatorPanel(QWidget * parent = nullptr);

  // Called by RViz once the panel is added; here we grab RViz's ROS node and
  // create the service clients.
  void onInitialize() override;

private Q_SLOTS:
  void onHomingClicked();
  void onEnableToggled(bool checked);

private:
  // Refresh the enable button's label/colour to reflect `enabled` (no signal).
  void updateEnableButton(bool enabled);
  void setStatus(const QString & text, bool ok);

  // Reset all RViz displays (equivalent to the "Reset" button) so MoveIt's
  // planning state re-syncs to the robot's current (homed) pose.
  void resetDisplays();

  QPushButton * homing_btn_{nullptr};
  QPushButton * enable_btn_{nullptr};
  QLabel *      status_label_{nullptr};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr homing_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_client_;
};

}  // namespace r0192_rviz_plugins
