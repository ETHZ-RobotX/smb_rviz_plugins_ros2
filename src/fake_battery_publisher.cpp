#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <chrono>
#include <memory>
#include <cstdlib>

using namespace std::chrono_literals;

class FakeBatteryPublisher : public rclcpp::Node {
public:
    FakeBatteryPublisher() : Node("fake_battery_publisher"), battery_percentage_(1.0), is_charging_(false) {
        // Parse parameters
        this->declare_parameter<double>("initial_percentage", 1.0);
        this->declare_parameter<bool>("start_charging", false);
        
        battery_percentage_ = this->get_parameter("initial_percentage").as_double();
        is_charging_ = this->get_parameter("start_charging").as_bool();

        // Create publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::BatteryState>("/battery_status", 10);
        
        // Create timer
        timer_ = this->create_wall_timer(1000ms, std::bind(&FakeBatteryPublisher::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Fake battery publisher started");
    }

private:
    void timer_callback() {
        auto message = sensor_msgs::msg::BatteryState();
        message.header.stamp = this->now();
        message.percentage = battery_percentage_;
        message.voltage = 24.0 * battery_percentage_; // Simulate 24V battery
        message.present = true;
        
        if(is_charging_) {
            message.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
            battery_percentage_ += 0.05; // Charge 5% per second
        } else {
            message.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
            battery_percentage_ -= 0.02; // Discharge 2% per second
        }

        // Clamp percentage between 0 and 1
        battery_percentage_ = std::clamp(battery_percentage_, 0.0, 1.0);
        
        // Randomly toggle charging state when reaching limits
        if(battery_percentage_ >= 0.95) {
            is_charging_ = false;
        } else if(battery_percentage_ <= 0.15 && rand() % 100 < 30) { // 30% chance to start charging when low
            is_charging_ = true;
        }

        publisher_->publish(message);
        
        RCLCPP_DEBUG(this->get_logger(), "Published battery status: %.1f%% (%s)", 
                    battery_percentage_ * 100, 
                    is_charging_ ? "Charging" : "Discharging");
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr publisher_;
    double battery_percentage_;
    bool is_charging_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeBatteryPublisher>());
    rclcpp::shutdown();
    return 0;
} 