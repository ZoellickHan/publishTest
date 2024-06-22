#include "rclcpp/rclcpp.hpp"
#include "rxtx_test/rxtx_Node.hpp"

namespace serial_driver
{
TestNode::TestNode(const rclcpp::NodeOptions & options) : Node("rxtx_test_node", options)
{
    gimbal_msg_sub_ = this->create_subscription<msg_interfaces::msg::GimbalMsg>( "/gimbal_command", rclcpp::SensorDataQoS(),
    std::bind(&TestNode::debug1,this,std::placeholders::_1 ));

    sentry_gimbal_msg_sub_ = this->create_subscription<msg_interfaces::msg::SentryGimbalMsg>("/sentry_gimbal",rclcpp::SensorDataQoS(),
    std::bind(&TestNode::debug2,this, std::placeholders::_1));

    RCLCPP_WARN(get_logger(),"Test Node...");

}

void TestNode::debug1(msg_interfaces::msg::GimbalMsg::SharedPtr msg) 
{
    RCLCPP_WARN(get_logger(),"receive");
    return;
}

void TestNode::debug2( msg_interfaces::msg::SentryGimbalMsg::SharedPtr msg) 
{
    RCLCPP_WARN(get_logger(),"receive");
    return;
}
}
