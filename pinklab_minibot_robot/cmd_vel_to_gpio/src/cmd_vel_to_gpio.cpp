#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>

class CmdVelToGPIO : public rclcpp::Node
{
public:
    CmdVelToGPIO()
        : Node("cmd_vel_to_gpio_node")
    {
        RCLCPP_INFO(this->get_logger(), "cmd_vel_to_gpio node has been started.");

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&CmdVelToGPIO::cmdVelCallback, this, std::placeholders::_1));

        left_motor_pub_ = this->create_publisher<std_msgs::msg::Float64>("gpio/set_left_motor_speed", 10);
        right_motor_pub_ = this->create_publisher<std_msgs::msg::Float64>("gpio/set_right_motor_speed", 10);

        RCLCPP_INFO(this->get_logger(), "Subscriptions and publishers have been initialized.");
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received cmd_vel message: linear.x=%.2f, angular.z=%.2f", msg->linear.x, msg->angular.z);

        auto left_motor_msg = std_msgs::msg::Float64();
        auto right_motor_msg = std_msgs::msg::Float64();

        // 간단한 모델: 왼쪽과 오른쪽 모터 속도를 선형 및 각속도로 계산
        left_motor_msg.data = msg->linear.x - msg->angular.z;
        right_motor_msg.data = msg->linear.x + msg->angular.z;

        RCLCPP_INFO(this->get_logger(), "Publishing left_motor_msg: %.2f, right_motor_msg: %.2f", left_motor_msg.data, right_motor_msg.data);

        left_motor_pub_->publish(left_motor_msg);
        right_motor_pub_->publish(right_motor_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_motor_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_motor_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CmdVelToGPIO>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
