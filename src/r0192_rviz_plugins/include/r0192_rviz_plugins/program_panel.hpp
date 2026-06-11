#pragma once

#include <rviz_common/panel.hpp>

#include <QComboBox>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <r0192_interfaces/action/execute_program.hpp>
#include <r0192_interfaces/msg/robot_state.hpp>

namespace r0192_rviz_plugins
{

class YamlHighlighter;

// ============================================================================
// ProgramPanel
//
// Operator/runtime panel for R0192 YAML programs — the counterpart of the
// VS Code engineering workflow (doku/program_ide_plan.md). Strictly
// runtime-only: load, run, stop, progress. Programs are NEVER edited here
// (the view is read-only); editing is VS Code's job.
//
// All business logic lives in the r0192_program_executor backend; this panel
// is a pure client of the /execute_program action:
//   - file picker over the programs directory (combo + browse)
//   - read-only YAML view with syntax highlighting
//   - live highlight of the currently executing step (action feedback)
//   - Run sends the goal, Stop cancels it (the executor stops the motion via
//     MoveGroupInterface::stop() and returns the state machine to HOLD)
//
// The UI is state-driven like the JogPanel: it mirrors the authoritative
// /robot_state (latched) and enables Run only when starting a program is
// possible (HOLD, or already MOVEIT). State transitions themselves are done
// by the EXECUTOR via /set_robot_state — the panel never touches the state
// machine.
//
// All ROS calls are asynchronous; RViz spins its node on the GUI thread, so
// the callbacks may touch Qt widgets directly.
// ============================================================================
class ProgramPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit ProgramPanel(QWidget * parent = nullptr);

  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

private Q_SLOTS:
  void onRefreshClicked();
  void onBrowseClicked();
  void onFileSelected(int index);
  void onRunClicked();
  void onStopClicked();

private:
  using ExecuteProgram = r0192_interfaces::action::ExecuteProgram;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ExecuteProgram>;

  void refreshFileList(const QString & keep_selected = QString());
  bool loadProgramText(const QString & path);   // (re)read the file into the view
  void indexStepLines();                        // map step index -> document line
  void highlightStep(int step_index);           // -1 clears the highlight
  void onFeedback(const std::shared_ptr<const ExecuteProgram::Feedback> & fb);
  void onResult(const GoalHandle::WrappedResult & result);
  void onRobotState(uint8_t state);
  void updateUiForState();
  void setStatus(const QString & text, bool ok);
  QString selectedFilePath() const;

  // --- Widgets ---
  QComboBox * file_combo_{nullptr};
  QPushButton * refresh_btn_{nullptr};
  QPushButton * browse_btn_{nullptr};
  QPlainTextEdit * program_view_{nullptr};
  YamlHighlighter * highlighter_{nullptr};
  QPushButton * run_btn_{nullptr};
  QPushButton * stop_btn_{nullptr};
  QLabel * progress_label_{nullptr};
  QLabel * state_label_{nullptr};
  QLabel * status_label_{nullptr};

  // --- Program/run state ---
  QString programs_dir_;            // persisted in the rviz config
  std::vector<int> step_lines_;     // document line of each step's "- " item
  bool running_{false};
  GoalHandle::SharedPtr goal_handle_;
  uint8_t robot_state_{r0192_interfaces::msg::RobotState::DISABLED};

  // --- ROS ---
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<ExecuteProgram>::SharedPtr action_client_;
  rclcpp::Subscription<r0192_interfaces::msg::RobotState>::SharedPtr state_sub_;
};

}  // namespace r0192_rviz_plugins
