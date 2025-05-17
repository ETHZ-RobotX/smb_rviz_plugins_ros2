#include "smb_rviz_plugins/smb_power_motor/smb_batt_mot_panel.hpp"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPainter>
#include <QPen>
#include <iostream>
#include <rviz_common/display_context.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <QDir>

namespace smb_rviz_plugins {

SmbBatteryMotPanel::SmbBatteryMotPanel(QWidget *parent) : Panel(parent) 
{
    QVBoxLayout* layout = new QVBoxLayout;
    battery_name_= new QLabel("Motor Battery");
    battery_name_->setStyleSheet("font-weight: bold");
    layout->addWidget(battery_name_);

    QHBoxLayout* layout_indicator = new QHBoxLayout;

    battery_icon_ = new QLabel();
    setIcon(":/battery/battery_warning.svg");
    layout_indicator->addWidget(battery_icon_);

    battery_text_ = new QLabel("No Data");
    layout_indicator->addWidget(battery_text_);

    layout_indicator->addStretch();

    layout->addLayout(layout_indicator);

    setLayout(layout);
}

SmbBatteryMotPanel::~SmbBatteryMotPanel()
{
    if (battery_sub_) {
        battery_sub_.reset();
    }
}

// void SmbBatteryMotPanel::onInitialize()
// {
//     // Get the ROS node from RViz context instead of creating our own
//     auto node_abstract = getDisplayContext()->getRosNodeAbstraction().lock();
//     node_ = node_abstract->get_raw_node();
    
//     // Create the topic property - without directly attaching the slot
//     battery_topic_property_ = new rviz_common::properties::StringProperty(
//         "Topic", "/battery_status",
//         "sensor_msgs/BatteryState topic to subscribe to",
//         this);
    
//     // Connect the property's changed signal to our slot using Qt5 style
//     connect(battery_topic_property_, &rviz_common::properties::Property::changed,
//             this, &SmbBatteryMotPanel::updateTopic);
    
//     // Subscribe to the topic
//     updateTopic();
    
//     // Add debugging
//     RCLCPP_INFO(node_->get_logger(), "Battery panel initialized and subscribed to: %s", 
//                 battery_topic_property_->getStdString().c_str());
// }

void SmbBatteryMotPanel::onInitialize()
{
    // Get the ROS node from RViz context instead of creating our own
    auto node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
    if (!node_abstraction) {
        RCLCPP_ERROR(rclcpp::get_logger("SmbBatteryMotPanel"), "Failed to get ROS node abstraction");
        return;
    }
    node_ = node_abstraction->get_raw_node();
    
    // Create the topic property (ensure spelling is correct)
    battery_topic_property_ = new rviz_common::properties::StringProperty(
        "Topic", "/battery_status",
        "sensor_msgs/BatteryState topic to subscribe to",
        nullptr);  // or 0 instead of 'this'

    // Connect the property's changed signal to our updateTopic slot using new Qt5 syntax
    connect(battery_topic_property_, &rviz_common::properties::Property::changed,
            this, &SmbBatteryMotPanel::updateTopic);
    
    // Subscribe/update the topic subscription
    updateTopic();
    
    // Log for debugging
    RCLCPP_INFO(node_->get_logger(), "Battery panel initialized and subscribed to: %s", 
                battery_topic_property_->getStdString().c_str());
}


// Interesting feature, this allows to change the topic of the panel at runtime, nice! 
// /battery_status is just a placeholder, it can be changed to any topic that publishes a sensor_msgs/BatteryState message
void SmbBatteryMotPanel::updateTopic()
{
    unsubscribe();
    subscribe();
}

void SmbBatteryMotPanel::subscribe()
{
    if (!battery_topic_property_ || !node_) {
        RCLCPP_ERROR(rclcpp::get_logger("smb_battery_panel"), 
                    "Failed to subscribe: %s", 
                    !battery_topic_property_ ? "No topic property" : "No node");
        return;
    }
    
    const std::string topic = battery_topic_property_->getStdString();
    if (topic.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "Empty topic name");
        return;
    }
    
    try {
        battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
            topic, 10, 
            std::bind(&SmbBatteryMotPanel::batteryMsgCallback, this, std::placeholders::_1));
        RCLCPP_INFO(node_->get_logger(), "Subscribed to: %s", topic.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Error subscribing to topic: %s", e.what());
    }
}

void SmbBatteryMotPanel::unsubscribe()
{
    if (battery_sub_) {
        battery_sub_.reset();
    }
}

void SmbBatteryMotPanel::batteryMsgCallback(const sensor_msgs::msg::BatteryState::ConstSharedPtr msg)
{
    // Update UI elements with message data
    setPercentage(msg->percentage);
    setVoltage(msg->voltage);
    
    if (!msg->present) {
        setBatteryStatus(BatteryStatus::Missing);
    } else {
        switch (msg->power_supply_status) {
            case sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING:
                setBatteryStatus(BatteryStatus::Charging);
                break;
            case sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING:
            case sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING:
                setBatteryStatus(BatteryStatus::Discharging);
                break;
            default:
                setBatteryStatus(BatteryStatus::Unknown);
        }
    }
}

void SmbBatteryMotPanel::setBatteryStatus(BatteryStatus status){
    battery_status_ = status;
    updateWidgets();
}

void SmbBatteryMotPanel::setPercentage(double percentage){
    percentage_ = percentage;
    updateWidgets();
}

void SmbBatteryMotPanel::setVoltage(double voltage){
    voltage_ = voltage;
    updateWidgets();
}

void SmbBatteryMotPanel::setIcon(const QString &icon_name)
{
    QPixmap pixmap(icon_name);
    if (pixmap.isNull()) {
        RCLCPP_ERROR(rclcpp::get_logger("SmbBatteryMotPanel"), 
                     "Failed to load icon: %s", icon_name.toStdString().c_str());
        // Try printing all available resources for debugging
        RCLCPP_INFO(rclcpp::get_logger("SmbBatteryMotPanel"), 
                    "Available resource paths: %s", 
                    QDir(":/").entryList().join(", ").toStdString().c_str());
        return;
    }
    
    const auto& font_metrics = battery_icon_->fontMetrics();
    auto icon_width = font_metrics.averageCharWidth() * 6;
    auto icon_height = font_metrics.height() * 2;
    battery_icon_->setPixmap(pixmap.scaled(icon_width, icon_height, Qt::KeepAspectRatio));
}

void SmbBatteryMotPanel::updateWidgets(){
    double percentage = percentage_ * 100;
    battery_text_->setText(QString("%1% (%2 V)").arg(QString::number(percentage, 'f', 2)).arg(QString::number(voltage_, 'f', 2)));
    
    // Fix the format string - use %% to print a literal % character
    RCLCPP_INFO(rclcpp::get_logger("SmbBatteryMotPanel"), 
                "Updating widget: status=%d, percentage=%.2f%%, voltage=%.2fV", 
                static_cast<int>(battery_status_), percentage, voltage_);
    
    switch(battery_status_)
    {
        case BatteryStatus::Unknown:
            setIcon(":/battery/icons/battery_warning.svg");
            break;
        case BatteryStatus::Charging:
            setIcon(":/battery/icons/battery_charge.svg");
            break;
        case BatteryStatus::Discharging:
        case BatteryStatus::NotCharging:
            if(percentage_ < 0.1) {
                setIcon(":/battery/icons/battery_0.svg");
            } else if(percentage_ < 0.2) {
                setIcon(":/battery/icons/battery_1.svg");
            } else if(percentage_ < 0.3) {
                setIcon(":/battery/icons/battery_2.svg");
            } else if(percentage_ < 0.4) {
                setIcon(":/battery/icons/battery_3.svg");
            } else if(percentage_ < 0.5) {
                setIcon(":/battery/icons/battery_4.svg");
            } else if(percentage_ < 0.6) {
                setIcon(":/battery/icons/battery_5.svg");
            } else if(percentage_ < 0.7) {
                setIcon(":/battery/icons/battery_6.svg");
            } else if(percentage_ < 0.8) {
                setIcon(":/battery/icons/battery_7.svg");
            } else if(percentage_ < 0.9) {
                setIcon(":/battery/icons/battery_8.svg");
            } else if(percentage_ < 1.0) {
                setIcon(":/battery/icons/battery_9.svg");
            } else {
                setIcon(":/battery/icons/battery_full.svg");
            }
            break;
        case BatteryStatus::Missing:
            setIcon(":/battery/icons/battery_warning.svg");
            break;
    }
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(smb_rviz_plugins::SmbBatteryMotPanel, rviz_common::Panel)