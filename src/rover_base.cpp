#include <chrono>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rover_interfaces/msg/pwm_array.hpp"

class WheelSpeedCalculator : public rclcpp::Node
{
    public:
        WheelSpeedCalculator()
        : Node("wheel_speed_calculator")
        {
            this->declare_parameter("pwm_array_topic_name", "/motors_pwm");
            this->declare_parameter("twist_topic_name", "/cmd_vel");
            this->declare_parameter("track_width", 893.83);
            this->declare_parameter("max_vel", 2924);

            subscription_twist =
                this->create_subscription<geometry_msgs::msg::Twist>(
                    this->get_parameter("twist_topic_name").as_string(),
                    rclcpp::SensorDataQoS(),
                    std::bind(&WheelSpeedCalculator::twist_callback, this, std::placeholders::_1)
                );

            publisher_ =
                this->create_publisher<rover_interfaces::msg::PwmArray>(
                    this->get_parameter("pwm_array_topic_name").as_string(),
                    rclcpp::SensorDataQoS()
                );
        }
    private:
        double vel_left = 0;
        double vel_right = 0;
        int linear_scaler = 1000;

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_twist;
        rclcpp::Publisher<rover_interfaces::msg::PwmArray>::SharedPtr publisher_;

        void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            std::cout << "Received twist message" << std::endl;
            std::cout << "Linear x: " << msg->linear.x << std::endl;
            std::cout << "Angular z: " << msg->angular.z << std::endl;
            double track_width = this->get_parameter("track_width").as_double();
            vel_left = (msg->linear.x * linear_scaler) - (msg->angular.z * track_width / 2);
            vel_right = (msg->linear.x * linear_scaler) + (msg->angular.z * track_width / 2);

            auto message = rover_interfaces::msg::PwmArray();
            message.pwm0 = mapToPwm(vel_left);
            message.pwm1 = mapToPwm(vel_left);
            message.pwm2 = mapToPwm(vel_left);
            message.pwm3 = mapToPwm(vel_right);
            message.pwm4 = mapToPwm(vel_right);
            message.pwm5 = mapToPwm(vel_right);

            publisher_->publish(message);
        }

        int16_t mapToPwm(double vel)
        {
            int max_vel = this->get_parameter("max_vel").as_int();
            return (int16_t) (vel * 255 / max_vel);
        }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelSpeedCalculator>());
    rclcpp::shutdown();
    return 0;
}