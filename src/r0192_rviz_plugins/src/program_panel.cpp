#include "r0192_rviz_plugins/program_panel.hpp"

#include <rviz_common/display_context.hpp>

#include <cmath>

#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QRegularExpression>
#include <QSyntaxHighlighter>
#include <QTextBlock>
#include <QTextStream>
#include <QVBoxLayout>

namespace r0192_rviz_plugins
{

namespace
{
constexpr const char * kActionName = "/execute_program";

const char * stateName(uint8_t s)
{
  using RS = r0192_interfaces::msg::RobotState;
  switch (s) {
    case RS::DISABLED: return "DISABLED";
    case RS::HOLD:     return "HOLD";
    case RS::JOG:      return "JOG";
    case RS::MOVEIT:   return "MOVEIT";
    case RS::HOMING:   return "HOMING";
    default:           return "UNKNOWN";
  }
}

const char * stepStatusName(uint8_t s)
{
  using FB = r0192_interfaces::action::ExecuteProgram::Feedback;
  switch (s) {
    case FB::STATUS_LOADING:  return "loading";
    case FB::STATUS_PLANNING: return "planning";
    case FB::STATUS_MOVING:   return "moving";
    case FB::STATUS_WAITING:  return "waiting";
    case FB::STATUS_PAUSED:   return "paused";
    default:                  return "?";
  }
}
}  // namespace

// ============================================================================
// Minimal YAML syntax highlighting for the read-only program view.
// ============================================================================
class YamlHighlighter : public QSyntaxHighlighter
{
public:
  explicit YamlHighlighter(QTextDocument * doc) : QSyntaxHighlighter(doc)
  {
    QTextCharFormat key;
    key.setForeground(QColor("#1565c0"));
    key.setFontWeight(QFont::Bold);
    rules_.push_back({QRegularExpression(R"(^\s*-?\s*[A-Za-z_][\w]*\s*(?=:))"), key});

    QTextCharFormat number;
    number.setForeground(QColor("#6a1b9a"));
    rules_.push_back({QRegularExpression(R"(\b-?\d+(\.\d+)?\b)"), number});

    QTextCharFormat str;
    str.setForeground(QColor("#2e7d32"));
    rules_.push_back({QRegularExpression(R"("[^"]*")"), str});

    QTextCharFormat comment;
    comment.setForeground(QColor("#9e9e9e"));
    comment.setFontItalic(true);
    rules_.push_back({QRegularExpression(R"(#.*$)"), comment});
  }

protected:
  void highlightBlock(const QString & text) override
  {
    for (const auto & rule : rules_) {
      auto it = rule.pattern.globalMatch(text);
      while (it.hasNext()) {
        const auto m = it.next();
        setFormat(m.capturedStart(), m.capturedLength(), rule.format);
      }
    }
  }

private:
  struct Rule
  {
    QRegularExpression pattern;
    QTextCharFormat format;
  };
  std::vector<Rule> rules_;
};

// ============================================================================
// ProgramPanel
// ============================================================================
ProgramPanel::ProgramPanel(QWidget * parent)
: rviz_common::Panel(parent),
  programs_dir_(QDir::homePath() + "/roboticarm_r0192_ws/programs")
{
  auto * root = new QVBoxLayout;

  // --- File picker row: combo over programs_dir + refresh + browse ---
  auto * file_row = new QHBoxLayout;
  file_combo_ = new QComboBox;
  file_combo_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  refresh_btn_ = new QPushButton("⟳");
  refresh_btn_->setFixedWidth(32);
  refresh_btn_->setToolTip("Re-scan the programs directory");
  browse_btn_ = new QPushButton("…");
  browse_btn_->setFixedWidth(32);
  browse_btn_->setToolTip("Open a program file from another location");
  file_row->addWidget(file_combo_);
  file_row->addWidget(refresh_btn_);
  file_row->addWidget(browse_btn_);
  root->addLayout(file_row);

  // --- Read-only program view (editing happens in VS Code, never here) ---
  program_view_ = new QPlainTextEdit;
  program_view_->setReadOnly(true);
  program_view_->setLineWrapMode(QPlainTextEdit::NoWrap);
  QFont mono("monospace");
  mono.setStyleHint(QFont::TypeWriter);
  program_view_->setFont(mono);
  program_view_->setPlaceholderText("No program loaded.\n\nPrograms are written in "
                                    "VS Code (programs/program_*.yaml);\nthis panel "
                                    "only loads and runs them.");
  highlighter_ = new YamlHighlighter(program_view_->document());
  root->addWidget(program_view_, 1);

  // --- Run / Pause / Stop ---
  auto * btn_row = new QHBoxLayout;
  run_btn_ = new QPushButton("Run");
  run_btn_->setStyleSheet(
    "QPushButton:enabled { background-color: #2e7d32; color: white; font-weight: bold; }");
  pause_btn_ = new QPushButton("Pause (after step)");
  pause_btn_->setToolTip("Finish the current step, then hold before the next one. "
                         "Resume continues the program.");
  pause_btn_->setEnabled(false);
  stop_btn_ = new QPushButton("Stop");
  stop_btn_->setStyleSheet(
    "QPushButton:enabled { background-color: #c62828; color: white; font-weight: bold; }");
  stop_btn_->setEnabled(false);
  btn_row->addWidget(run_btn_);
  btn_row->addWidget(pause_btn_);
  btn_row->addWidget(stop_btn_);
  root->addLayout(btn_row);

  // --- Run mode: stop at each point vs. Pilz "blend through" ---
  blend_check_ = new QCheckBox("Blend through (Pilz)");
  blend_check_->setToolTip(
    "Off: stop at each point (each move planned separately).\n"
    "On: consecutive moves run as one Pilz sequence with c_dis blending. "
    "Required for circ steps.");
  root->addWidget(blend_check_);

  // --- Simulate (dry run): preview only, the real arm does not move ---
  dry_run_check_ = new QCheckBox("Simulate (dry run) — preview only, arm does not move");
  dry_run_check_->setToolTip(
    "On: plan the whole program and animate it as the RViz ghost only — the real "
    "arm stays put and the robot state is untouched (works from any state).\n"
    "Off: run on the real arm.");
  root->addWidget(dry_run_check_);

  // --- Speed override (applies from the NEXT step; mirrors /program_override) ---
  auto * override_row = new QHBoxLayout;
  override_row->addWidget(new QLabel("Override"));
  override_slider_ = new QSlider(Qt::Horizontal);
  override_slider_->setRange(10, 100);
  override_slider_->setValue(100);
  override_slider_->setTracking(false);   // emit valueChanged only on release
  override_slider_->setToolTip("Speed override — applies from the next step");
  override_value_ = new QLabel("100 %");
  override_value_->setMinimumWidth(44);
  override_row->addWidget(override_slider_);
  override_row->addWidget(override_value_);
  root->addLayout(override_row);

  // --- Progress + state + status ---
  progress_label_ = new QLabel("—");
  progress_label_->setStyleSheet("font-family: monospace;");
  root->addWidget(progress_label_);
  state_label_ = new QLabel("State: …");
  root->addWidget(state_label_);
  status_label_ = new QLabel("Waiting for robot state …");
  status_label_->setWordWrap(true);
  root->addWidget(status_label_);

  // --- Point management (phase 4): list + teach + delete. Thin client of the
  //     backend point services; explicit values are edited in VS Code only. ---
  auto * points_box = new QGroupBox("Points (points.yaml)");
  auto * points_layout = new QVBoxLayout;
  points_list_ = new QListWidget;
  points_list_->setSelectionMode(QAbstractItemView::SingleSelection);
  points_list_->setMaximumHeight(120);
  points_layout->addWidget(points_list_);

  auto * teach_row = new QHBoxLayout;
  teach_name_ = new QLineEdit;
  teach_name_->setPlaceholderText("point name");
  teach_type_ = new QComboBox;
  teach_type_->addItem("joint", r0192_interfaces::srv::TeachPoint::Request::TYPE_JOINT);
  teach_type_->addItem("pose", r0192_interfaces::srv::TeachPoint::Request::TYPE_POSE);
  teach_btn_ = new QPushButton("Teach");
  teach_btn_->setToolTip("Store the CURRENT robot position under this name "
                         "(requires HOLD or JOG)");
  teach_row->addWidget(teach_name_, 1);
  teach_row->addWidget(teach_type_);
  teach_row->addWidget(teach_btn_);
  points_layout->addLayout(teach_row);

  auto * points_btn_row = new QHBoxLayout;
  points_refresh_btn_ = new QPushButton("Refresh");
  delete_point_btn_ = new QPushButton("Delete selected");
  points_btn_row->addWidget(points_refresh_btn_);
  points_btn_row->addWidget(delete_point_btn_);
  points_layout->addLayout(points_btn_row);

  points_box->setLayout(points_layout);
  root->addWidget(points_box);

  setLayout(root);

  connect(refresh_btn_, &QPushButton::clicked, this, &ProgramPanel::onRefreshClicked);
  connect(browse_btn_, &QPushButton::clicked, this, &ProgramPanel::onBrowseClicked);
  connect(file_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &ProgramPanel::onFileSelected);
  connect(run_btn_, &QPushButton::clicked, this, &ProgramPanel::onRunClicked);
  // Dry run relaxes the state gating (preview from any state) — re-evaluate Run.
  connect(dry_run_check_, &QCheckBox::toggled, this, [this](bool) { updateUiForState(); });
  connect(pause_btn_, &QPushButton::clicked, this, &ProgramPanel::onPauseResumeClicked);
  connect(stop_btn_, &QPushButton::clicked, this, &ProgramPanel::onStopClicked);
  connect(override_slider_, &QSlider::valueChanged, this, &ProgramPanel::onOverrideChanged);
  connect(points_refresh_btn_, &QPushButton::clicked, this, &ProgramPanel::onPointsRefreshClicked);
  connect(teach_btn_, &QPushButton::clicked, this, &ProgramPanel::onTeachClicked);
  connect(delete_point_btn_, &QPushButton::clicked, this, &ProgramPanel::onDeletePointClicked);
  connect(points_list_, &QListWidget::itemSelectionChanged, this,
          [this]() { delete_point_btn_->setEnabled(points_list_->currentItem() != nullptr); });

  updateUiForState();
}

void ProgramPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  action_client_ = rclcpp_action::create_client<ExecuteProgram>(node_, kActionName);
  teach_client_ = node_->create_client<r0192_interfaces::srv::TeachPoint>("/teach_point");
  list_client_ = node_->create_client<r0192_interfaces::srv::ListPoints>("/list_points");
  delete_client_ = node_->create_client<r0192_interfaces::srv::DeletePoint>("/delete_point");
  pause_client_ = node_->create_client<std_srvs::srv::Trigger>("/pause_program");
  resume_client_ = node_->create_client<std_srvs::srv::Trigger>("/resume_program");
  override_client_ =
    node_->create_client<r0192_interfaces::srv::SetProgramOverride>("/set_program_override");

  // The slider follows the authoritative (latched) override value instead of
  // assuming its own request succeeded — same pattern as the state-driven UI.
  override_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
    "/program_override", rclcpp::QoS(1).transient_local(),
    [this](const std_msgs::msg::Float32::ConstSharedPtr msg) {
      const int percent = static_cast<int>(std::lround(msg->data * 100.0f));
      override_slider_->blockSignals(true);
      override_slider_->setValue(percent);
      override_slider_->blockSignals(false);
      override_value_->setText(QString::number(percent) + " %");
    });

  state_sub_ = node_->create_subscription<r0192_interfaces::msg::RobotState>(
    "/robot_state", rclcpp::QoS(1).transient_local(),
    [this](const r0192_interfaces::msg::RobotState::ConstSharedPtr msg) {
      onRobotState(msg->state);
    });

  refreshFileList();
}

// ----------------------------------------------------------------------------
// Panel config persistence (programs directory survives RViz restarts)
// ----------------------------------------------------------------------------
void ProgramPanel::save(rviz_common::Config config) const
{
  Panel::save(config);
  config.mapSetValue("ProgramsDir", programs_dir_);
}

void ProgramPanel::load(const rviz_common::Config & config)
{
  Panel::load(config);
  QString dir;
  if (config.mapGetString("ProgramsDir", &dir) && !dir.isEmpty()) {
    programs_dir_ = dir;
  }
}

// ----------------------------------------------------------------------------
// File handling
// ----------------------------------------------------------------------------
void ProgramPanel::refreshFileList(const QString & keep_selected)
{
  const QString previous =
    keep_selected.isEmpty() ? selectedFilePath() : keep_selected;

  file_combo_->blockSignals(true);
  file_combo_->clear();
  const QDir dir(programs_dir_);
  for (const auto & name : dir.entryList({"*.yaml", "*.yml"}, QDir::Files, QDir::Name)) {
    if (name == "points.yaml") continue;  // the point database is not a program
    file_combo_->addItem(name, dir.absoluteFilePath(name));
  }
  // Keep a browsed file (outside the directory) selectable.
  if (!previous.isEmpty() && file_combo_->findData(previous) < 0 &&
      QFileInfo::exists(previous)) {
    file_combo_->addItem(QFileInfo(previous).fileName() + "  (external)", previous);
  }
  const int idx = file_combo_->findData(previous);
  file_combo_->setCurrentIndex(idx >= 0 ? idx : (file_combo_->count() > 0 ? 0 : -1));
  file_combo_->blockSignals(false);

  onFileSelected(file_combo_->currentIndex());
}

void ProgramPanel::onRefreshClicked()
{
  refreshFileList();
  setStatus(QString("Found %1 program(s) in %2").arg(file_combo_->count()).arg(programs_dir_),
            true);
}

void ProgramPanel::onBrowseClicked()
{
  const QString path = QFileDialog::getOpenFileName(
    this, "Open R0192 program", programs_dir_, "YAML programs (*.yaml *.yml)");
  if (path.isEmpty()) {
    return;
  }
  // Browsing into another directory makes it the new working directory.
  const QFileInfo info(path);
  if (info.absoluteDir().exists() && info.absoluteDir() != QDir(programs_dir_)) {
    programs_dir_ = info.absolutePath();
    Q_EMIT configChanged();
  }
  refreshFileList(info.absoluteFilePath());
}

void ProgramPanel::onFileSelected(int index)
{
  highlightStep(-1);
  progress_label_->setText("—");
  if (index < 0) {
    program_view_->clear();
    step_lines_.clear();
    updateUiForState();
    return;
  }
  loadProgramText(file_combo_->itemData(index).toString());
  updateUiForState();
}

bool ProgramPanel::loadProgramText(const QString & path)
{
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    program_view_->clear();
    step_lines_.clear();
    setStatus("Cannot read " + path, false);
    return false;
  }
  QTextStream in(&file);
  program_view_->setPlainText(in.readAll());
  indexStepLines();
  return true;
}

// Maps step index -> document line of its "- " list item. The executor counts
// steps in YAML order, and the schema only has one sequence ('steps:', with
// inline joint arrays), so every "- " item line after 'steps:' is one step.
void ProgramPanel::indexStepLines()
{
  step_lines_.clear();
  static const QRegularExpression steps_key(R"(^steps\s*:)");
  static const QRegularExpression item(R"(^\s*-\s)");
  const auto * doc = program_view_->document();
  bool in_steps = false;
  for (int line = 0; line < doc->blockCount(); ++line) {
    const QString text = doc->findBlockByNumber(line).text();
    if (steps_key.match(text).hasMatch()) {
      in_steps = true;
      continue;
    }
    if (in_steps && item.match(text).hasMatch()) {
      step_lines_.push_back(line);
    }
  }
}

void ProgramPanel::highlightStep(int step_index)
{
  QList<QTextEdit::ExtraSelection> selections;
  if (step_index >= 0 && step_index < static_cast<int>(step_lines_.size())) {
    auto * doc = program_view_->document();
    const int start = step_lines_[step_index];
    const int end = (step_index + 1 < static_cast<int>(step_lines_.size()))
                      ? step_lines_[step_index + 1]
                      : doc->blockCount();
    for (int line = start; line < end; ++line) {
      const QTextBlock block = doc->findBlockByNumber(line);
      if (!block.isValid()) break;
      QTextEdit::ExtraSelection sel;
      sel.format.setBackground(QColor(255, 213, 79, 110));   // soft amber
      sel.format.setProperty(QTextFormat::FullWidthSelection, true);
      sel.cursor = QTextCursor(block);
      selections.append(sel);
    }
    QTextCursor cursor(doc->findBlockByNumber(start));
    program_view_->setTextCursor(cursor);
    program_view_->ensureCursorVisible();
  }
  program_view_->setExtraSelections(selections);
}

// ----------------------------------------------------------------------------
// Run / Stop (action client)
// ----------------------------------------------------------------------------
QString ProgramPanel::selectedFilePath() const
{
  const int idx = file_combo_->currentIndex();
  return idx >= 0 ? file_combo_->itemData(idx).toString() : QString();
}

void ProgramPanel::onRunClicked()
{
  const QString path = selectedFilePath();
  if (path.isEmpty() || running_) {
    return;
  }
  if (!action_client_->action_server_is_ready()) {
    setStatus("Executor unavailable (/execute_program) — is the backend running?", false);
    return;
  }
  // Re-read so the view matches what the executor will load (the file may have
  // just been edited in VS Code).
  if (!loadProgramText(path)) {
    return;
  }

  ExecuteProgram::Goal goal;
  goal.program_path = path.toStdString();
  goal.blend = blend_check_->isChecked();
  goal.dry_run = dry_run_check_->isChecked();

  rclcpp_action::Client<ExecuteProgram>::SendGoalOptions options;
  options.goal_response_callback = [this](const GoalHandle::SharedPtr & gh) {
    if (!gh) {
      running_ = false;
      goal_handle_.reset();
      setStatus("Goal rejected (a program may already be running).", false);
      updateUiForState();
      return;
    }
    goal_handle_ = gh;
    setStatus("Program started.", true);
  };
  options.feedback_callback =
    [this](GoalHandle::SharedPtr, const std::shared_ptr<const ExecuteProgram::Feedback> fb) {
      onFeedback(fb);
    };
  options.result_callback = [this](const GoalHandle::WrappedResult & result) {
    onResult(result);
  };

  running_ = true;
  highlightStep(-1);
  progress_label_->setText("starting …");
  setStatus("Sending goal …", true);
  updateUiForState();
  action_client_->async_send_goal(goal, options);
}

void ProgramPanel::onStopClicked()
{
  if (!goal_handle_) {
    return;
  }
  setStatus("Stopping — cancelling current motion …", false);
  action_client_->async_cancel_goal(goal_handle_);
}

void ProgramPanel::onPauseResumeClicked()
{
  auto & client = paused_ ? resume_client_ : pause_client_;
  if (!client->service_is_ready()) {
    setStatus("Pause/resume service unavailable — is the executor running?", false);
    return;
  }
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  client->async_send_request(
    req,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      const auto resp = future.get();
      setStatus(QString::fromStdString(resp->message), resp->success);
      // The paused flag itself follows the STATUS_PAUSED action feedback.
    });
}

void ProgramPanel::onOverrideChanged(int percent)
{
  override_value_->setText(QString::number(percent) + " %");
  if (!override_client_->service_is_ready()) {
    setStatus("Override service unavailable — is the executor running?", false);
    return;
  }
  auto req = std::make_shared<r0192_interfaces::srv::SetProgramOverride::Request>();
  req->override = static_cast<float>(percent) / 100.0f;
  override_client_->async_send_request(
    req,
    [this](rclcpp::Client<r0192_interfaces::srv::SetProgramOverride>::SharedFuture future) {
      const auto resp = future.get();
      // Slider/label sync happens via the latched /program_override topic.
      setStatus(QString::fromStdString(resp->message), resp->success);
    });
}

void ProgramPanel::onFeedback(const std::shared_ptr<const ExecuteProgram::Feedback> & fb)
{
  const bool paused_now = (fb->status == ExecuteProgram::Feedback::STATUS_PAUSED);
  if (paused_now != paused_) {
    paused_ = paused_now;
    pause_btn_->setText(paused_ ? "Resume" : "Pause (after step)");
  }
  if (fb->total_steps == 0) {
    progress_label_->setText("loading program …");
    return;
  }
  progress_label_->setText(QString("%1 %2 / %3 — %4 [%5]")
                             .arg(paused_now ? "Paused before step" : "Step")
                             .arg(fb->current_step + 1)
                             .arg(fb->total_steps)
                             .arg(QString::fromStdString(fb->step_label))
                             .arg(stepStatusName(fb->status)));
  highlightStep(static_cast<int>(fb->current_step));
}

void ProgramPanel::onResult(const GoalHandle::WrappedResult & result)
{
  running_ = false;
  paused_ = false;
  pause_btn_->setText("Pause (after step)");
  goal_handle_.reset();
  highlightStep(-1);

  const QString msg = QString::fromStdString(result.result->message);
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      progress_label_->setText(QString("Finished (%1 steps).").arg(result.result->steps_completed));
      setStatus(msg, true);
      break;
    case rclcpp_action::ResultCode::CANCELED:
      progress_label_->setText("Stopped.");
      setStatus(msg, false);
      break;
    default:
      progress_label_->setText("Failed.");
      setStatus(msg.isEmpty() ? "Program aborted." : msg, false);
      break;
  }
  updateUiForState();
}

// ----------------------------------------------------------------------------
// Point management (thin client of /list_points, /teach_point, /delete_point)
// ----------------------------------------------------------------------------
void ProgramPanel::onPointsRefreshClicked()
{
  refreshPointList();
}

void ProgramPanel::refreshPointList()
{
  if (!list_client_ || !list_client_->service_is_ready()) {
    setStatus("Point services unavailable — is the executor running?", false);
    return;
  }
  auto req = std::make_shared<r0192_interfaces::srv::ListPoints::Request>();
  list_client_->async_send_request(
    req,
    [this](rclcpp::Client<r0192_interfaces::srv::ListPoints>::SharedFuture future) {
      const auto resp = future.get();
      points_list_->clear();
      if (!resp->success) {
        setStatus(QString::fromStdString(resp->message), false);
        return;
      }
      using Teach = r0192_interfaces::srv::TeachPoint::Request;
      for (std::size_t i = 0; i < resp->names.size(); ++i) {
        const char * type =
          (i < resp->types.size() && resp->types[i] == Teach::TYPE_POSE) ? "pose" : "joint";
        auto * item = new QListWidgetItem(
          QString("%1   [%2]").arg(QString::fromStdString(resp->names[i])).arg(type));
        item->setData(Qt::UserRole, QString::fromStdString(resp->names[i]));
        points_list_->addItem(item);
      }
      delete_point_btn_->setEnabled(points_list_->currentItem() != nullptr);
    });
}

void ProgramPanel::onTeachClicked()
{
  const QString name = teach_name_->text().trimmed();
  if (name.isEmpty()) {
    setStatus("Enter a point name first.", false);
    return;
  }
  sendTeach(name, static_cast<uint8_t>(teach_type_->currentData().toUInt()), false);
}

void ProgramPanel::sendTeach(const QString & name, uint8_t type, bool overwrite)
{
  if (!teach_client_->service_is_ready()) {
    setStatus("Teach service unavailable — is the executor running?", false);
    return;
  }
  auto req = std::make_shared<r0192_interfaces::srv::TeachPoint::Request>();
  req->name = name.toStdString();
  req->type = type;
  req->overwrite = overwrite;
  teach_client_->async_send_request(
    req,
    [this, name, type](rclcpp::Client<r0192_interfaces::srv::TeachPoint>::SharedFuture future) {
      const auto resp = future.get();
      if (resp->success) {
        setStatus(QString::fromStdString(resp->message), true);
        refreshPointList();
        return;
      }
      // Existing name: confirm before replacing (the backend rejects unless
      // 'overwrite' is set).
      if (QString::fromStdString(resp->message).contains("already exists")) {
        const auto answer = QMessageBox::question(
          this, "Overwrite point?",
          QString("Point '%1' already exists. Overwrite it with the current position?")
            .arg(name));
        if (answer == QMessageBox::Yes) {
          sendTeach(name, type, true);
          return;
        }
      }
      setStatus(QString::fromStdString(resp->message), false);
    });
}

void ProgramPanel::onDeletePointClicked()
{
  auto * item = points_list_->currentItem();
  if (!item) {
    return;
  }
  const QString name = item->data(Qt::UserRole).toString();
  const auto answer = QMessageBox::question(
    this, "Delete point?",
    QString("Delete point '%1' from points.yaml? Programs referencing it will "
            "fail to load.").arg(name));
  if (answer != QMessageBox::Yes) {
    return;
  }
  if (!delete_client_->service_is_ready()) {
    setStatus("Delete service unavailable — is the executor running?", false);
    return;
  }
  auto req = std::make_shared<r0192_interfaces::srv::DeletePoint::Request>();
  req->name = name.toStdString();
  delete_client_->async_send_request(
    req,
    [this](rclcpp::Client<r0192_interfaces::srv::DeletePoint>::SharedFuture future) {
      const auto resp = future.get();
      setStatus(QString::fromStdString(resp->message), resp->success);
      if (resp->success) refreshPointList();
    });
}

// ----------------------------------------------------------------------------
// State-driven UI
// ----------------------------------------------------------------------------
void ProgramPanel::onRobotState(uint8_t state)
{
  robot_state_ = state;
  // First state message means the stack (incl. executor) is up — load the list.
  if (!points_loaded_once_) {
    points_loaded_once_ = true;
    refreshPointList();
  }
  updateUiForState();
}

void ProgramPanel::updateUiForState()
{
  using RS = r0192_interfaces::msg::RobotState;
  state_label_->setText(QString("State: %1").arg(stateName(robot_state_)));

  // A real run needs a state from which the executor can enter MOVEIT (HOLD;
  // MOVEIT itself is a no-op transition). A dry run only plans + animates, so it
  // works from any state (the arm never moves).
  const bool dry = dry_run_check_->isChecked();
  const bool startable = dry || robot_state_ == RS::HOLD || robot_state_ == RS::MOVEIT;
  run_btn_->setEnabled(!running_ && startable && !selectedFilePath().isEmpty());
  run_btn_->setToolTip(
    startable
      ? (dry ? "Simulate the program — ghost preview only, the arm does not move"
             : "Execute the selected program (HOLD -> MOVEIT -> HOLD)")
      : "Enable the servos first (state must be HOLD) — or tick Simulate to preview");
  pause_btn_->setEnabled(running_);
  stop_btn_->setEnabled(running_);

  // Switching files or the run mode mid-run would desync the step highlight.
  file_combo_->setEnabled(!running_);
  browse_btn_->setEnabled(!running_);
  refresh_btn_->setEnabled(!running_);
  blend_check_->setEnabled(!running_);
  dry_run_check_->setEnabled(!running_);

  // Teaching captures the current position — only meaningful from a defined
  // resting state (HOLD after moves/jog, or directly while jogging).
  const bool teachable = (robot_state_ == RS::HOLD || robot_state_ == RS::JOG);
  teach_btn_->setEnabled(teachable);
  teach_btn_->setToolTip(teachable
                           ? "Store the CURRENT robot position under this name"
                           : "Teaching requires state HOLD or JOG");
  delete_point_btn_->setEnabled(points_list_->currentItem() != nullptr);
}

void ProgramPanel::setStatus(const QString & text, bool ok)
{
  status_label_->setText(text);
  status_label_->setStyleSheet(ok ? "color: #2e7d32;" : "color: #c62828;");
}

}  // namespace r0192_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(r0192_rviz_plugins::ProgramPanel, rviz_common::Panel)
