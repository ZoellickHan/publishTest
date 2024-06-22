#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>
#include <deque>
#include "protocol.hpp"
#include "rclcpp/rclcpp.hpp"

#include "msg_interfaces/msg/gimbal_msg.hpp"
#include "msg_interfaces/msg/sentry_gimbal_msg.hpp"
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
  
    rclcpp::Subscription<msg_interfaces::msg::SentryGimbalMsg>::SharedPtr sentry_gimbal_msg_sub_;
    rclcpp::Subscription<msg_interfaces::msg::GimbalMsg>::SharedPtr gimbal_msg_sub_;
    int bag_sum = 0;

};
}