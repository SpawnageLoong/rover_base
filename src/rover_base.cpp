#include <chrono>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "geometry_msgs/msg/twist.hpp"

class WheelSpeedCalculator : public rclcpp::Node
{
    public:
        WheelSpeedCalculator()
        : Node("wheel_speed_calculator")
        {
            this->declare_parameter("topic0_name", "/motors_pwm/motor0");
            this->declare_parameter("topic1_name", "/motors_pwm/motor1");
            this->declare_parameter("topic2_name", "/motors_pwm/motor2");
            this->declare_parameter("topic3_name", "/motors_pwm/motor3");
            this->declare_parameter("topic4_name", "/motors_pwm/motor4");
            this->declare_parameter("topic5_name", "/motors_pwm/motor5");
            this->declare_parameter("twist_topic_name", "/cmd_vel");
            this->declare_parameter("track_width", 600.0);

            subscription_twist =
                this->create_subscription<geometry_msgs::msg::Twist>(
                    this->get_parameter("twist_topic_name").as_string(),
                    rclcpp::SensorDataQoS(),
                    std::bind(&WheelSpeedCalculator::twist_callback, this, std::placeholders::_1)
                );

            publisher_0 =
                this->create_publisher<std_msgs::msg::UInt8>(
                    this->get_parameter("topic0_name").as_string(),
                    rclcpp::SensorDataQoS()
                );
            publisher_1 =
                this->create_publisher<std_msgs::msg::UInt8>(
                    this->get_parameter("topic1_name").as_string(),
                    rclcpp::SensorDataQoS()
                );
            publisher_2 =
                this->create_publisher<std_msgs::msg::UInt8>(
                    this->get_parameter("topic2_name").as_string(),
                    rclcpp::SensorDataQoS()
                );
            publisher_3 =
                this->create_publisher<std_msgs::msg::UInt8>(
                    this->get_parameter("topic3_name").as_string(),
                    rclcpp::SensorDataQoS()
                );
            publisher_4 =
                this->create_publisher<std_msgs::msg::UInt8>(
                    this->get_parameter("topic4_name").as_string(),
                    rclcpp::SensorDataQoS()
                );
            publisher_5 =
                this->create_publisher<std_msgs::msg::UInt8>(
                    this->get_parameter("topic5_name").as_string(),
                    rclcpp::SensorDataQoS()
                );
        }
    private:
        double vel_left = 0;
        double vel_right = 0;

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_twist;
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr publisher_0;
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr publisher_1;
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr publisher_2;
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr publisher_3;
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr publisher_4;
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr publisher_5;

        void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            std::cout << "Received twist message" << std::endl;
            std::cout << "Linear x: " << msg->linear.x << std::endl;
            std::cout << "Angular z: " << msg->angular.z << std::endl;
            double track_width = this->get_parameter("track_width").as_double();
            vel_left = msg->linear.x - (msg->angular.z * track_width / 2);
            vel_right = msg->linear.x + (msg->angular.z * track_width / 2);

            auto message_left = std_msgs::msg::UInt8();
            message_left.data = mapToPwm(vel_left);
            auto message_right = std_msgs::msg::UInt8();
            message_right.data = mapToPwm(vel_right);

            publisher_0->publish(message_left);
            publisher_1->publish(message_left);
            publisher_2->publish(message_left);
            publisher_3->publish(message_right);
            publisher_4->publish(message_right);
            publisher_5->publish(message_right);
        }

        uint8_t mapToPwm(double vel)
        {
            return (uint8_t) (vel * 255 / 100);
        }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelSpeedCalculator>());
    rclcpp::shutdown();
    return 0;
}