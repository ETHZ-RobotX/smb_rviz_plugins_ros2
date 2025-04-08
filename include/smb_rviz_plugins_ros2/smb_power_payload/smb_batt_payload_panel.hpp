#pragma once

#include <memory>
#include <string>

#include <QWidget>

#include "rviz_common/panel.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

class QLabel;
class QLineEdit;
class QProgressBar;
class QTimer;

namespace smb_rviz_plugins_ros2
{

class SmbBatteryPayloadPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  // Constructor and destructor
  explicit SmbBatteryPayloadPanel(QWidget* parent = nullptr);
  ~SmbBatteryPayloadPanel() override;

  // Called when the panel is initialized
  void onInitialize() override;

  // Setup the subscription with the current topic
  void setupSubscription();

  // Save panel configuration
  void save(rviz_common::Config config) const override;
  
  // Load panel configuration
  void load(const rviz_common::Config& config) override;

protected Q_SLOTS:
  // Update the topic when the user clicks the Update Topic button
  void updateTopic();
  
  // Check if we're receiving messages
  void checkConnection();
  
  // Update the UI with the latest battery data
  void updateBatteryDisplay();

Q_SIGNALS:
  // Signal emitted when new battery data is received
  void batteryDataReceived();

private:
  // ROS related members
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr subscriber_;
  QString battery_topic_;
  
  // UI elements
  QLineEdit* topic_edit_;
  QLabel* voltage_label_;
  QLabel* current_label_;
  QLabel* percentage_label_;
  QProgressBar* charge_bar_;
  QTimer* connection_timer_;
  
  // Battery state values
  double voltage_;
  double current_;
  double percentage_;
};

}  // namespace smb_rviz_plugins_ros2
