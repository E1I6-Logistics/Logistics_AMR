#include "logistics_amr_rviz_plugins/multi_robot_control_panel.hpp"

#include <QComboBox>
#include <QFrame>
#include <QGridLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#include "pluginlib/class_list_macros.hpp"

namespace logistics_amr_rviz_plugins
{

MultiRobotControlPanel::MultiRobotControlPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * title = new QLabel(tr("Multi-Robot Goal Control"));
  QFont title_font = title->font();
  title_font.setBold(true);
  title_font.setPointSize(title_font.pointSize() + 1);
  title->setFont(title_font);

  auto * instruction = new QLabel(
    tr("1. Select a robot\n2. Use the 2D Goal Pose tool"));
  instruction->setWordWrap(true);

  robot_combo_ = new QComboBox;
  robot_combo_->addItem(tr("Waiting for dispatcher..."));
  robot_combo_->setEnabled(false);

  select_button_ = new QPushButton(tr("Select Robot"));
  select_button_->setEnabled(false);
  connect(select_button_, &QPushButton::clicked, this, &MultiRobotControlPanel::selectRobot);

  cancel_button_ = new QPushButton(tr("Cancel Goal"));
  cancel_button_->setEnabled(false);
  connect(cancel_button_, &QPushButton::clicked, this, &MultiRobotControlPanel::cancelGoal);

  selected_robot_label_ = new QLabel(tr("None"));
  status_label_ = new QLabel(tr("Waiting"));
  status_label_->setAlignment(Qt::AlignCenter);
  status_label_->setMinimumHeight(28);
  detail_label_ = new QLabel(tr("Waiting for Goal Dispatcher"));
  detail_label_->setWordWrap(true);

  auto * grid = new QGridLayout;
  grid->addWidget(new QLabel(tr("Target Robot")), 0, 0);
  grid->addWidget(robot_combo_, 0, 1);
  grid->addWidget(select_button_, 1, 0, 1, 2);
  grid->addWidget(new QLabel(tr("Selected")), 2, 0);
  grid->addWidget(selected_robot_label_, 2, 1);
  grid->addWidget(new QLabel(tr("Goal State")), 3, 0);
  grid->addWidget(status_label_, 3, 1);
  grid->addWidget(new QLabel(tr("Detail")), 4, 0);
  grid->addWidget(detail_label_, 4, 1);
  grid->addWidget(cancel_button_, 5, 0, 1, 2);

  auto * separator = new QFrame;
  separator->setFrameShape(QFrame::HLine);
  separator->setFrameShadow(QFrame::Sunken);

  auto * layout = new QVBoxLayout;
  layout->addWidget(title);
  layout->addWidget(instruction);
  layout->addWidget(separator);
  layout->addLayout(grid);
  layout->addStretch();
  setLayout(layout);
  setStatusStyle(QStringLiteral("waiting"));

  node_ = std::make_shared<rclcpp::Node>("multi_robot_control_panel");
  selected_robot_publisher_ = node_->create_publisher<std_msgs::msg::String>(
    "/multi_robot/selected_robot", 10);
  cancel_publisher_ = node_->create_publisher<std_msgs::msg::String>(
    "/multi_robot/cancel_goal", 10);

  auto robot_list_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  robot_list_subscription_ = node_->create_subscription<std_msgs::msg::String>(
    "/multi_robot/robots",
    robot_list_qos,
    [this](const std_msgs::msg::String::SharedPtr message) {
      robotListCallback(message);
    });
  status_subscription_ = node_->create_subscription<std_msgs::msg::String>(
    "/multi_robot/goal_status",
    10,
    [this](const std_msgs::msg::String::SharedPtr message) {
      goalStatusCallback(message);
    });

  spin_timer_ = new QTimer(this);
  connect(spin_timer_, &QTimer::timeout, this, &MultiRobotControlPanel::spinSome);
  spin_timer_->start(50);
}

MultiRobotControlPanel::~MultiRobotControlPanel()
{
  if (spin_timer_ != nullptr) {
    spin_timer_->stop();
  }
  status_subscription_.reset();
  robot_list_subscription_.reset();
  cancel_publisher_.reset();
  selected_robot_publisher_.reset();
  node_.reset();
}

void MultiRobotControlPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString robot;
  if (config.mapGetString("selected_robot", &robot)) {
    saved_robot_ = robot;
  }
}

void MultiRobotControlPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("selected_robot", selected_robot_);
}

void MultiRobotControlPanel::selectRobot()
{
  if (!robot_combo_->isEnabled() || robot_combo_->currentText().isEmpty()) {
    return;
  }

  selected_robot_ = robot_combo_->currentText();
  std_msgs::msg::String message;
  message.data = selected_robot_.toStdString();
  selected_robot_publisher_->publish(message);
  updateSelectedRobotView();
  Q_EMIT configChanged();
}

void MultiRobotControlPanel::cancelGoal()
{
  if (selected_robot_.isEmpty()) {
    return;
  }

  std_msgs::msg::String message;
  message.data = selected_robot_.toStdString();
  cancel_publisher_->publish(message);
  detail_label_->setText(tr("Cancel requested"));
}

void MultiRobotControlPanel::spinSome()
{
  if (node_) {
    rclcpp::spin_some(node_);
  }
}

void MultiRobotControlPanel::robotListCallback(
  const std_msgs::msg::String::SharedPtr message)
{
  QJsonParseError error;
  const auto document = QJsonDocument::fromJson(
    QByteArray::fromStdString(message->data), &error);
  if (error.error != QJsonParseError::NoError || !document.isObject()) {
    detail_label_->setText(tr("Invalid robot list from dispatcher"));
    return;
  }

  const auto robots = document.object().value("robots").toArray();
  robot_combo_->clear();
  for (const auto & robot_value : robots) {
    const auto robot = robot_value.toString();
    if (!robot.isEmpty()) {
      robot_combo_->addItem(robot);
    }
  }

  const bool has_robots = robot_combo_->count() > 0;
  robot_combo_->setEnabled(has_robots);
  select_button_->setEnabled(has_robots);
  if (!has_robots) {
    detail_label_->setText(tr("Dispatcher reported no robots"));
    return;
  }

  const QString preferred_robot =
    !selected_robot_.isEmpty() ? selected_robot_ : saved_robot_;
  const int preferred_index = robot_combo_->findText(preferred_robot);
  if (preferred_index >= 0) {
    robot_combo_->setCurrentIndex(preferred_index);
  }
  detail_label_->setText(tr("Choose a robot, then press Select Robot"));
}

void MultiRobotControlPanel::goalStatusCallback(
  const std_msgs::msg::String::SharedPtr message)
{
  QJsonParseError error;
  const auto document = QJsonDocument::fromJson(
    QByteArray::fromStdString(message->data), &error);
  if (error.error != QJsonParseError::NoError || !document.isObject()) {
    detail_label_->setText(QString::fromStdString(message->data));
    return;
  }

  const auto object = document.object();
  const QString robot = object.value("robot").toString();
  const QString state = object.value("state").toString();
  const QString detail = object.value("detail").toString();
  if (!robot.isEmpty()) {
    state_by_robot_[robot] = state;
    detail_by_robot_[robot] = detail;
  }
  if (state == QStringLiteral("selected") && !robot.isEmpty()) {
    selected_robot_ = robot;
    const int index = robot_combo_->findText(robot);
    if (index >= 0) {
      robot_combo_->setCurrentIndex(index);
    }
  }
  updateSelectedRobotView();
}

void MultiRobotControlPanel::updateSelectedRobotView()
{
  if (selected_robot_.isEmpty()) {
    selected_robot_label_->setText(tr("None"));
    status_label_->setText(tr("Waiting"));
    detail_label_->setText(tr("Select a robot first"));
    cancel_button_->setEnabled(false);
    setStatusStyle(QStringLiteral("waiting"));
    return;
  }

  selected_robot_label_->setText(selected_robot_);
  const QString state = state_by_robot_.value(selected_robot_, QStringLiteral("selected"));
  const QString detail = detail_by_robot_.value(selected_robot_);
  status_label_->setText(state.toUpper());
  detail_label_->setText(
    detail.isEmpty() ? tr("Use 2D Goal Pose to send a goal") : detail);
  cancel_button_->setEnabled(
    state == QStringLiteral("sending") ||
    state == QStringLiteral("accepted") ||
    state == QStringLiteral("active") ||
    state == QStringLiteral("canceling"));
  setStatusStyle(state);
}

void MultiRobotControlPanel::setStatusStyle(const QString & state)
{
  QString color = QStringLiteral("#6b7280");
  if (state == QStringLiteral("accepted") || state == QStringLiteral("active") ||
    state == QStringLiteral("succeeded"))
  {
    color = QStringLiteral("#15803d");
  } else if (state == QStringLiteral("sending") || state == QStringLiteral("canceling")) {
    color = QStringLiteral("#a16207");
  } else if (state == QStringLiteral("rejected") || state == QStringLiteral("aborted")) {
    color = QStringLiteral("#b91c1c");
  }
  status_label_->setStyleSheet(
    QStringLiteral("QLabel { color: white; background: %1; border-radius: 4px; padding: 4px; }")
    .arg(color));
}

}  // namespace logistics_amr_rviz_plugins

PLUGINLIB_EXPORT_CLASS(
  logistics_amr_rviz_plugins::MultiRobotControlPanel,
  rviz_common::Panel)
