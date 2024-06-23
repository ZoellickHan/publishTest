//ros
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>
//msg_interfaces
#include "msg_interfaces/msg/gimbal_msg.hpp"
#include "msg_interfaces/msg/sentry_gimbal_msg.hpp"
#include "msg_interfaces/msg/chassis_command.hpp"
#include "msg_interfaces/msg/gimbal_command.hpp"
#include "msg_interfaces/msg/sentry_gimbal_command.hpp"
//other
#include "protocol.hpp"
namespace serial_driver
{
class TestNode : public rclcpp::Node
{
public:
    TestNode(const rclcpp::NodeOptions & options);
    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
private:

    void debug1(msg_interfaces::msg::GimbalMsg::SharedPtr msg);
    void debug2(msg_interfaces::msg::SentryGimbalMsg::SharedPtr msg);
    void pub();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<msg_interfaces::msg::SentryGimbalMsg>::SharedPtr sentry_gimbal_msg_sub_;
    rclcpp::Subscription<msg_interfaces::msg::GimbalMsg>::SharedPtr gimbal_msg_sub_;

    rclcpp::Publisher<msg_interfaces::msg::GimbalCommand>::SharedPtr gimbal_command_pub_;
    rclcpp::Publisher<msg_interfaces::msg::ChassisCommand>::SharedPtr chassis_command_pub_;
    rclcpp::Publisher<msg_interfaces::msg::SentryGimbalCommand>::SharedPtr sentry_gimbal_command_pub_;
    int pkg_sum = 0;
    std::thread pub_thread;

};
}