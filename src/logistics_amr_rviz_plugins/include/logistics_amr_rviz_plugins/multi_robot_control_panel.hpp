#ifndef LOGISTICS_AMR_RVIZ_PLUGINS__MULTI_ROBOT_CONTROL_PANEL_HPP_
#define LOGISTICS_AMR_RVIZ_PLUGINS__MULTI_ROBOT_CONTROL_PANEL_HPP_

#include <memory>
#include <string>

#include <QMap>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "std_msgs/msg/string.hpp"

class QComboBox;
class QLabel;
class QPushButton;
class QTimer;

namespace logistics_amr_rviz_plugins
{

class MultiRobotControlPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit MultiRobotControlPanel(QWidget * parent = nullptr);
  ~MultiRobotControlPanel() override;

  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void selectRobot();
  void cancelGoal();
  void spinSome();

private:
  void robotListCallback(const std_msgs::msg::String::SharedPtr message);
  void goalStatusCallback(const std_msgs::msg::String::SharedPtr message);
  void updateSelectedRobotView();
  void setStatusStyle(const QString & state);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr selected_robot_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cancel_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_list_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscription_;

  QComboBox * robot_combo_{nullptr};
  QPushButton * select_button_{nullptr};
  QPushButton * cancel_button_{nullptr};
  QLabel * selected_robot_label_{nullptr};
  QLabel * status_label_{nullptr};
  QLabel * detail_label_{nullptr};
  QTimer * spin_timer_{nullptr};

  QString selected_robot_;
  QString saved_robot_;
  QMap<QString, QString> state_by_robot_;
  QMap<QString, QString> detail_by_robot_;
};

}  // namespace logistics_amr_rviz_plugins

#endif  // LOGISTICS_AMR_RVIZ_PLUGINS__MULTI_ROBOT_CONTROL_PANEL_HPP_
