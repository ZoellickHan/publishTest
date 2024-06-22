#include "rclcpp/rclcpp.hpp"
#include "rxtx_test/rxtx_Node.hpp"
#include <chrono>

namespace serial_driver
{
TestNode::TestNode(const rclcpp::NodeOptions & options) : rclcpp::Node("rxtx_test_node", options)
{
    gimbal_msg_sub_ = this->create_subscription<msg_interfaces::msg::GimbalMsg>( "/gimbal_msg", rclcpp::SensorDataQoS(),
    std::bind(&TestNode::debug1,this,std::placeholders::_1 ));

    sentry_gimbal_msg_sub_ = this->create_subscription<msg_interfaces::msg::SentryGimbalMsg>("/sentry_gimbal_msg",rclcpp::SensorDataQoS(),
    std::bind(&TestNode::debug2,this, std::placeholders::_1));

    RCLCPP_WARN(get_logger(),"Test Node");
    

}

void TestNode::debug1(msg_interfaces::msg::GimbalMsg::SharedPtr msg) 
{
    this->bag_sum ++;
    std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time = end - start;

    RCLCPP_WARN(get_logger(),"bagsum = %d , bag rate : %f ,time :%f ,gimbal msg bullet speed = %f \n",
    bag_sum,double(bag_sum)/time.count(),time.count(),msg->bullet_speed);
    // printf("gimbal msg bullet speed = %f \n",msg->bullet_speed);
    return;
}

void TestNode::debug2( msg_interfaces::msg::SentryGimbalMsg::SharedPtr msg) 
{
    bag_sum ++;
    std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time = end - start;
    RCLCPP_WARN(get_logger(),"bagsum = %d , bag rate : %f ,time :%f ,gimbal msg bullet speed = %f \n",
    bag_sum,double(bag_sum)/time.count(),time.count(),msg->bullet_speed);
    RCLCPP_WARN(get_logger(),"gimbal msg bullet speed = %f \n",msg->bullet_speed);
    return;
}
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(serial_driver::TestNode)
