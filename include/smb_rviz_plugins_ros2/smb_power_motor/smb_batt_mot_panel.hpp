#ifndef SMB_BATT_MOT_PANEL_HPP__
#define SMB_BATT_MOT_PANEL_HPP__

#include <rviz_common/panel.hpp>
#include <rviz_common/properties/string_property.hpp>

#include <QWidget>
#include <QLabel>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

namespace smb_rviz_plugins_ros2
{
class SmbBatteryMotPanel : public rviz_common::Panel
{
    Q_OBJECT

    public:
        SmbBatteryMotPanel(QWidget *parent = nullptr);
        ~SmbBatteryMotPanel();
        
        enum class BatteryStatus {
            Unknown,
            Charging,
            Discharging,
            Missing,
            NotCharging
        };
        Q_ENUM(BatteryStatus);

        void setBatteryStatus(BatteryStatus status);
        void setPercentage(double percentage);
        void setVoltage(double voltage);

    private Q_SLOTS:
        void updateTopic();

    private:
        void setIcon(const QString &icon_name);
        void updateWidgets();
        void batteryMsgCallback(const sensor_msgs::msg::BatteryState::ConstSharedPtr msg);
        void onInitialize() override;
        void subscribe();
        void unsubscribe();

        BatteryStatus battery_status_{BatteryStatus::Unknown};
        double percentage_{0.0};
        double voltage_{0.0};

        QLabel* battery_name_;
        QLabel* battery_icon_;
        QLabel* battery_text_;

        rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
        rclcpp::Node::SharedPtr node_;

        rviz_common::properties::StringProperty* battery_topic_property_{nullptr};
    };

} // namespace smb_rviz_plugins_ros2
#endif //MB_BATT_MOT_PANEL_HPP 