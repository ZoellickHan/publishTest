#include "rclcpp/rclcpp.hpp"
#include "rxtx_test/rxtx_Node.hpp"

TestNode::TestNode(const rclcpp::NodeOptions & options) : Node("rm_serial_driver", options)
{
        gimbal_msg_sub_ = this->create_subscription<msg_interfaces::msg::GimbalMsg>( "/gimbal_command", rclcpp::SensorDataQoS(),
        std::bind(&TestNode::debug1,this,std::placeholders::_1 ));

        sentry_gimbal_msg_sub_ = this->create_subscription<msg_interfaces::msg::SentryGimbalMsg>("/sentry_gimbal",rclcpp::SensorDataQoS(),
        std::bind(&TestNode::debug2,this, std::placeholders::_1));
    while(true)
    {
    }
}

void TestNode::debug1(msg_interfaces::msg::GimbalMsg::SharedPtr msg) 
{
    return;
}

void TestNode::debug2( msg_interfaces::msg::SentryGimbalMsg::SharedPtr msg) 
{
    return;
}
