#include "smb_rviz_plugins_ros2/smb_power_payload/smb_batt_payload_panel.hpp"

#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QProgressBar>
#include <QTimer>

#include "rviz_common/display_context.hpp"

namespace smb_rviz_plugins_ros2 {

SmbBatteryPayloadPanel::SmbBatteryPayloadPanel(QWidget* parent)
: rviz_common::Panel(parent),
  battery_topic_("/smb/payload_battery"),
  voltage_(0.0),
  current_(0.0),
  percentage_(0.0)
{
  // Set up the layout
  QGridLayout* layout = new QGridLayout;
  
  // Add topic selection
  layout->addWidget(new QLabel("Topic:"), 0, 0);
  
  topic_edit_ = new QLineEdit;
  topic_edit_->setText(battery_topic_);
  layout->addWidget(topic_edit_, 0, 1);
  
  QPushButton* topic_button = new QPushButton("Update Topic");
  layout->addWidget(topic_button, 0, 2);
  connect(topic_button, SIGNAL(clicked()), this, SLOT(updateTopic()));
  
  // Add labels for battery information
  layout->addWidget(new QLabel("Voltage:"), 1, 0);
  voltage_label_ = new QLabel("0.0 V");
  layout->addWidget(voltage_label_, 1, 1);
  
  layout->addWidget(new QLabel("Current:"), 2, 0);
  current_label_ = new QLabel("0.0 A");
  layout->addWidget(current_label_, 2, 1);
  
  layout->addWidget(new QLabel("Charge:"), 3, 0);
  percentage_label_ = new QLabel("0%");
  layout->addWidget(percentage_label_, 3, 1);
  
  // Add progress bar for charge level
  charge_bar_ = new QProgressBar;
  charge_bar_->setRange(0, 100);
  charge_bar_->setValue(0);
  layout->addWidget(charge_bar_, 4, 0, 1, 3);
  
  setLayout(layout);
  
  // Set up a timer to periodically check connection status
  connection_timer_ = new QTimer(this);
  connect(connection_timer_, SIGNAL(timeout()), this, SLOT(checkConnection()));
  connection_timer_->start(2000);  // Check every 2 seconds
  
  // Connect our signal to our slot
  connect(this, &SmbBatteryPayloadPanel::batteryDataReceived, this, &SmbBatteryPayloadPanel::updateBatteryDisplay);
}

SmbBatteryPayloadPanel::~SmbBatteryPayloadPanel() {}

void SmbBatteryPayloadPanel::onInitialize()
{
  // Get the ROS node from the display context
  auto context = getDisplayContext();
  if (context) {
    node_ = context->getRosNodeAbstraction().lock()->get_raw_node();
    setupSubscription();
  }
}

void SmbBatteryPayloadPanel::setupSubscription()
{
  if (!node_) {
    return;
  }
  
  // Reset any existing subscription
  subscriber_.reset();
  
  // Create a new subscription - convert QString to std::string
  std::string topic_str = battery_topic_.toStdString();
  subscriber_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
    topic_str, rclcpp::QoS(10),
    [this](sensor_msgs::msg::BatteryState::SharedPtr msg) {
      // Update the stored values
      voltage_ = msg->voltage;
      current_ = msg->current;
      percentage_ = msg->percentage * 100.0;  // Convert to percentage
      
      // Update the UI from the main thread
      Q_EMIT batteryDataReceived();
    });
}

void SmbBatteryPayloadPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Topic", battery_topic_);
}

void SmbBatteryPayloadPanel::load(const rviz_common::Config& config)
{
  rviz_common::Panel::load(config);
  QString topic;
  if (config.mapGetString("Topic", &topic))
  {
    battery_topic_ = topic;
    topic_edit_->setText(topic);
    
    if (node_) {
      setupSubscription();
    }
  }
}

void SmbBatteryPayloadPanel::updateTopic()
{
  battery_topic_ = topic_edit_->text();
  setupSubscription();
}

void SmbBatteryPayloadPanel::checkConnection()
{
  // This could be enhanced to show status in the UI
}

void SmbBatteryPayloadPanel::updateBatteryDisplay()
{
  voltage_label_->setText(QString("%1 V").arg(voltage_, 0, 'f', 2));
  current_label_->setText(QString("%1 A").arg(current_, 0, 'f', 2));
  percentage_label_->setText(QString("%1%").arg(static_cast<int>(percentage_)));
  charge_bar_->setValue(static_cast<int>(percentage_));
  
  // Adjust color based on charge level
  if (percentage_ < 20.0) {
    charge_bar_->setStyleSheet("QProgressBar::chunk { background-color: red; }");
  } else if (percentage_ < 50.0) {
    charge_bar_->setStyleSheet("QProgressBar::chunk { background-color: yellow; }");
  } else {
    charge_bar_->setStyleSheet("QProgressBar::chunk { background-color: green; }");
  }
}

} // namespace smb_rviz_plugins_ros2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(smb_rviz_plugins_ros2::SmbBatteryPayloadPanel, rviz_common::Panel)
