#include <rclcpp/rclcpp.hpp>
// #include <rclcpp/logging.hpp>
// #include <rclcpp/publisher.hpp>
// #include <rclcpp/qos.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp/subscription.hpp>
// #include <rclcpp/utilities.hpp>
// #include <std_msgs/msg/string.hpp>
#include <deque>
#include "protocol.hpp"

class TestNode : public rclcpp :: Node
{
public:
    TestNode() : Node("test")
    {
        while(true)
        {
            gimbal_msg_sub_ = create_subscription<serial_driver::TwoCRC_GimbalMsg>( "gimbal_command", rclcpp::SensorDataQoS(),
            std::bind(debug1, this, std::placeholders::_1));

            sentry_gimbal_msg_sub_ = create_subscription<serial_driver::TwoCRC_SentryGimbalMsg>("sentry_gimbal",rclcpp::SensorDataQoS(),
            std::bind(debug2,this,std::placeholders::_1));

            chassis_cmd_ = create_publisher<serial_driver::TwoCRC_ChassisCommand>("chassis cmd publish",10);
            gimbal_cmd_  = create_publisher<serial_driver::TwoCRC_GimbalCommand>("gimbal cmd publish",10); 
            
            chassis_cmd_ -> publish(twoCRC_ChassisCommand);
            gimbal_cmd_ -> publish(twoCRC_GimbalCommand);
        }
    }
    ~TestNode();

private:

    void debug1();

    void debug2();
    // rclcpp::Publisher<serial_driver::TwoCRC_ChassisCommand>::SharedPtr chassis_cmd_;
    rclcpp::Publisher<auto_aim_interfaces::msg::
    rclcpp::Publisher<serial_driver::TwoCRC_GimbalCommand>::SharedPtr gimbal_cmd_;
    rclcpp::Subscription<serial_driver::TwoCRC_SentryGimbalMsg>::SharedPtr sentry_gimbal_msg_sub_;
    rclcpp::Subscription<serial_driver::TwoCRC_GimbalMsg>::SharedPtr gimbal_msg_sub_;
    
    serial_driver::TwoCRC_GimbalCommand   twoCRC_GimbalCommand;
    serial_driver::TwoCRC_ChassisCommand  twoCRC_ChassisCommand;  

};