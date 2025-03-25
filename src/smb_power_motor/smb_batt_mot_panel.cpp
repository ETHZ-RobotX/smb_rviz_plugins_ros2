#include "smb_rviz_plugins_ros2/smb_power_motor/smb_batt_mot_panel.hpp"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPainter>
#include <QPen>
#include <iostream>
#include <rviz_common/display_context.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/properties/string_property.hpp>

namespace smb_rviz_plugins_ros2 {

SmbBatteryMotPanel::SmbBatteryMotPanel(QWidget *parent) : Panel(parent) 
{
    // Initialize ROS2 components
    node_ = std::make_shared<rclcpp::Node>("smb_battery_panel");
    
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

void SmbBatteryMotPanel::onInitialize()
{
    battery_topic_property_ = new rviz_common::properties::StringProperty(
        "Topic", "/battery_status",
        "sensor_msgs/BatteryState topic to subscribe to",
        nullptr,  // No parent property since Panel isn't a Property
        SLOT(updateTopic()));
    
    subscribe();
}

void SmbBatteryMotPanel::updateTopic()
{
    unsubscribe();
    subscribe();
}

void SmbBatteryMotPanel::subscribe()
{
    try {
        battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
            battery_topic_property_->getStdString(),
            10,
            std::bind(&SmbBatteryMotPanel::batteryMsgCallback, this, std::placeholders::_1));
    } catch (const rclcpp::exceptions::InvalidTopicNameError& e) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Invalid topic name: " << e.what());
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

void SmbBatteryMotPanel::setIcon(const QString &path){
    QPixmap pixmap(path);
    const auto& font_metrics = battery_icon_->fontMetrics();
    auto icon_width = font_metrics.averageCharWidth() * 6;
    auto icon_height = font_metrics.height()*2;
    battery_icon_->setPixmap(pixmap.scaled(icon_width, icon_height, Qt::KeepAspectRatio));
}

void SmbBatteryMotPanel::updateWidgets(){
    double percentage = percentage_ * 100;
    battery_text_->setText(QString("%1% (%2 V)").arg(QString::number(percentage, 'f', 2)).arg(QString::number(voltage_, 'f', 2)));
    switch(battery_status_)
    {
        case BatteryStatus::Unknown:
            setIcon(":/battery/battery_warning.svg");
            break;
        case BatteryStatus::Charging:
            setIcon(":/battery/battery_charge.svg");
            break;
        case BatteryStatus::Discharging:
        case BatteryStatus::NotCharging:
            if(percentage_ < 0.1)
                setIcon(":/battery/battery_0.svg");
            else if(percentage_ < 0.2)
                setIcon(":/battery/battery_1.svg");
            else if(percentage_ < 0.3)
                setIcon(":/battery/battery_2.svg");
            else if(percentage_ < 0.4)
                setIcon(":/battery/battery_3.svg");
            else if(percentage_ < 0.5)
                setIcon(":/battery/battery_4.svg");
            else if(percentage_ < 0.6)
                setIcon(":/battery/battery_5.svg");
            else if(percentage_ < 0.7)
                setIcon(":/battery/battery_6.svg");
            else if(percentage_ < 0.8)
                setIcon(":/battery/battery_7.svg");
            else if(percentage_ < 0.9)
                setIcon(":/battery/battery_8.svg");
            else
                setIcon(":/battery/battery_full.svg");
            break;
        case BatteryStatus::Missing:
            setIcon(":/battery/battery_warning.svg");
            break;
    }
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(smb_rviz_plugins_ros2::SmbBatteryMotPanel, rviz_common::Panel)