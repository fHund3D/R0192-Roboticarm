#pragma once

#include <rviz_common/panel.hpp>

#include <QPushButton>
#include <QRadioButton>
#include <QLabel>
#include <QSlider>
#include <QTimer>

#include <array>
#include <map>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <r0192_interfaces/msg/robot_state.hpp>
#include <r0192_interfaces/srv/set_robot_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace rviz_common { class Display; }

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
// The panel is a CLIENT of the central state machine (robot_state_manager): it
// reflects /robot_state and requests transitions via /set_robot_state. Servos,
// MoveIt, Jog and Homing are mutually-exclusive states; the panel only enables
// the buttons whose transitions are valid from the current state. The Jog
// command streaming itself (command type + press-and-hold publishing) still
// lives here and runs only while the arm is in the JOG state. The speed slider
// scales the commanded velocity (Servo command_in_type = "unitless").
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
  void onValueTick();   // refresh the live joint-angle / TCP-pose readout
  void onHomingClicked();
  void onEnableToggled(bool checked);
  void onMoveItToggled(bool checked);
  void onEStopClicked();
  void onResetClicked();
  void onQueryGoalToggled(bool show);

private:
  enum class Mode { Joint, CartesianBase, Tool };

  void setMode(Mode mode);
  void updateDofLabels();
  void applyCommandType();          // switch Servo command type for current mode
  void setJogButtonsEnabled(bool enabled);
  void setStatus(const QString & text, bool ok);
  double speedFactor() const;       // slider -> [0, 1] unitless magnitude
  void resetDisplays();             // reset all RViz displays (post-homing)
  rviz_common::Display * findMotionPlanningDisplay();  // MoveIt display lookup

  // --- State-machine client ---
  void requestState(uint8_t target);          // call /set_robot_state
  void onRobotState(uint8_t state, const QString & status);  // /robot_state cb
  void updateUiForState();                     // sync buttons + jog stream to state

  // --- Widgets ---
  QRadioButton * mode_joint_{nullptr};
  QRadioButton * mode_base_{nullptr};
  QRadioButton * mode_tool_{nullptr};
  std::array<QPushButton *, 6> minus_btn_{};
  std::array<QPushButton *, 6> plus_btn_{};
  std::array<QLabel *, 6>      dof_label_{};
  std::array<QLabel *, 6>      value_label_{};   // live readout per DOF
  QSlider *      speed_slider_{nullptr};
  QLabel *       speed_value_{nullptr};
  QPushButton *  estop_btn_{nullptr};
  QPushButton *  reset_btn_{nullptr};
  QPushButton *  jog_active_btn_{nullptr};
  QPushButton *  moveit_btn_{nullptr};
  QPushButton *  homing_btn_{nullptr};
  QPushButton *  enable_btn_{nullptr};
  QPushButton *  query_goal_btn_{nullptr};
  QLabel *       status_label_{nullptr};

  QTimer * pub_timer_{nullptr};
  QTimer * value_timer_{nullptr};

  // --- Jog state ---
  Mode mode_{Mode::Joint};
  int    active_dof_{-1};    // -1 = no button held
  double active_sign_{0.0};

  // --- Robot state (mirrors /robot_state) ---
  uint8_t robot_state_{r0192_interfaces::msg::RobotState::DISABLED};
  uint8_t prev_robot_state_{r0192_interfaces::msg::RobotState::DISABLED};

  // --- ROS ---
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr      joint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr  switch_type_client_;
  rclcpp::Client<r0192_interfaces::srv::SetRobotState>::SharedPtr set_state_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr             estop_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr             reset_client_;
  rclcpp::Subscription<r0192_interfaces::msg::RobotState>::SharedPtr state_sub_;

  // --- Live readout sources ---
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr  joint_state_sub_;
  std::map<std::string, double> joint_pos_;   // latest /joint_states positions
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace r0192_rviz_plugins
