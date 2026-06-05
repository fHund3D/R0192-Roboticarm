#pragma once

#include <rviz_common/panel.hpp>

#include <QPushButton>
#include <QRadioButton>
#include <QLabel>
#include <QSlider>
#include <QTimer>

#include <array>

#include <rclcpp/rclcpp.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace r0192_rviz_plugins
{

// ============================================================================
// JogPanel
//
// Teach-pendant style jog panel for the R0192 arm, built on MoveIt Servo.
// Three jog modes share the same 6 x (- / +) buttons:
//
//   - "Achsen"             joint jogging          -> JointJog on
//                                                    /servo_node/delta_joint_cmds
//   - "Kartesisch (Basis)" Cartesian, base frame  -> TwistStamped (base_link)
//   - "Werkzeug (Tool)"    Cartesian, tool frame  -> TwistStamped (gripper_base)
//
// A master "Jog aktiv" toggle unpauses Servo (and selects the command type) so
// it takes over arm_controller; switching it off pauses Servo again and hands
// the controller back to move_group for normal MoveIt planning. The speed
// slider scales the commanded velocity (Servo command_in_type = "unitless").
//
// While a +/- button is held, a timer streams the command at a fixed rate
// (press-and-hold continuous jog). Releasing stops the stream; Servo halts the
// arm once the command goes stale.
//
// All ROS calls are asynchronous; RViz spins its node on the GUI thread, so the
// callbacks may touch Qt widgets directly.
// ============================================================================
class JogPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit JogPanel(QWidget * parent = nullptr);

  void onInitialize() override;

private Q_SLOTS:
  void onModeChanged();
  void onJogActiveToggled(bool checked);
  void onAxisPressed(int dof, double sign);
  void onAxisReleased();
  void onPublishTick();

private:
  enum class Mode { Joint, CartesianBase, Tool };

  void setMode(Mode mode);
  void updateDofLabels();
  void applyCommandType();          // switch Servo command type for current mode
  void callPause(bool pause);       // /servo_node/pause_servo
  void setJogButtonsEnabled(bool enabled);
  void setStatus(const QString & text, bool ok);
  double speedFactor() const;       // slider -> [0, 1] unitless magnitude

  // --- Widgets ---
  QRadioButton * mode_joint_{nullptr};
  QRadioButton * mode_base_{nullptr};
  QRadioButton * mode_tool_{nullptr};
  std::array<QPushButton *, 6> minus_btn_{};
  std::array<QPushButton *, 6> plus_btn_{};
  std::array<QLabel *, 6>      dof_label_{};
  QSlider *      speed_slider_{nullptr};
  QLabel *       speed_value_{nullptr};
  QPushButton *  jog_active_btn_{nullptr};
  QLabel *       status_label_{nullptr};

  QTimer * pub_timer_{nullptr};

  // --- Jog state ---
  Mode mode_{Mode::Joint};
  int    active_dof_{-1};    // -1 = no button held
  double active_sign_{0.0};

  // --- ROS ---
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr      joint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr  switch_type_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr              pause_client_;
};

}  // namespace r0192_rviz_plugins
