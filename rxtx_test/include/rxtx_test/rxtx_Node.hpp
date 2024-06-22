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

class TestNode : public rclcpp :: Node
{
public:
    TestNode(const rclcpp::NodeOptions & options);
    ~TestNode();

private:

    void debug1(msg_interfaces::msg::GimbalMsg::SharedPtr msg);

    void debug2(msg_interfaces::msg::SentryGimbalMsg::SharedPtr msg);
    // rclcpp::Publisher<acesserial_driver::TwoCRC_ChassisCommand>::SharedPtr chassis_cmd_;
    // rclcpp::Publisher<serial_driver::TwoCRC_GimbalCommand>::SharedPtr gimbal_cmd_;
    rclcpp::Subscription<msg_interfaces::msg::SentryGimbalMsg>::SharedPtr sentry_gimbal_msg_sub_;
    rclcpp::Subscription<msg_interfaces::msg::GimbalMsg>::SharedPtr gimbal_msg_sub_;
    
    serial_driver::TwoCRC_GimbalCommand   twoCRC_GimbalCommand;
    serial_driver::TwoCRC_ChassisCommand  twoCRC_ChassisCommand;  

};