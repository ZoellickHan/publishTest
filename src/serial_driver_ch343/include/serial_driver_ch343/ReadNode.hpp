#ifndef SERIAL_DRIVER_MY_NODE_HPP_
#define SERIAL_DRIVER_MY_NODE_HPP_

#define READER_BUFFER_SIZE 64
#define MAX_BUFFER_SIZE 2024
#define DECODE_BUFFER_SIZE 128
#define TRANSMIT_BUFFER 128

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <deque>
#include <thread>
#include <mutex>

#include "serial_driver_ch343/protocol.hpp"
#include "serial_driver_ch343/crc.hpp"
#include  "serialDriver.hpp"

#include "msg_interfaces/msg/gimbal_msg.hpp"
#include "msg_interfaces/msg/sentry_gimbal_msg.hpp"
#include "msg_interfaces/msg/chassis_command.hpp"
#include "msg_interfaces/msg/gimbal_command.hpp"
#include "msg_interfaces/msg/sentry_gimbal_command.hpp"
#include "msg_interfaces/msg/test_msg.hpp"
#include "msg_interfaces/msg/test_msg2.hpp"

#include <chrono>
#include <cstring>
namespace serial_driver
{

class ReadNode : public rclcpp::Node
{
public:
  ReadNode(const rclcpp::NodeOptions & options);
  std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

private:    
    //function
    PkgState decode();
    int receive();
    int transmit();
    void classify(uint8_t* data);
    void eraser();
    void GimbalCommand_CB(msg_interfaces::msg::GimbalCommand::SharedPtr msg);
    void ChassisCommand_CB(msg_interfaces::msg::ChassisCommand::SharedPtr msg);
    void SentryGimbalCommand_CB(msg_interfaces::msg::SentryGimbalCommand::SharedPtr msg);
    void test_CB(msg_interfaces::msg::TestMsg::SharedPtr msg);
    void test2_CB(msg_interfaces::msg::TestMsg2::SharedPtr msg);
    void rx();

    int trans_pkg_sum = 0;
    std::shared_ptr<SerialConfig> config;
    //  (2000000,8,false,StopBit::TWO,Parity::NONE);

    std::shared_ptr<Port> port ;
    // std::make_shared<Port>(config);

    std::deque<uint8_t> receive_buffer;
    std::deque<uint8_t> transmit_buffer;
    uint8_t decodeBuffer[DECODE_BUFFER_SIZE];
    uint8_t receiveBuffer[READER_BUFFER_SIZE];
    std::mutex transmit_mutex;
    std::thread tx_thread;
    std::thread rx_thread;

    //info
    rclcpp::Publisher<msg_interfaces::msg::GimbalMsg>::SharedPtr gimabal_msg_pub_;
    rclcpp::Publisher<msg_interfaces::msg::SentryGimbalMsg>::SharedPtr sentry_gimbal_msg_pub_;

    rclcpp::Subscription<msg_interfaces::msg::GimbalCommand>::SharedPtr gimbal_command_sub_;
    rclcpp::Subscription<msg_interfaces::msg::ChassisCommand>::SharedPtr chassis_command_sub_;
    rclcpp::Subscription<msg_interfaces::msg::SentryGimbalCommand>::SharedPtr sentry_gimbal_command_sub_;
    rclcpp::Subscription<msg_interfaces::msg::TestMsg>::SharedPtr acction_sub_;
    rclcpp::Subscription<msg_interfaces::msg::TestMsg2>::SharedPtr test_2_sub_;
    rclcpp::TimerBase::SharedPtr timer1_;
    rclcpp::TimerBase::SharedPtr timer2_;
    //protocol 
    Header   header;
    PkgState pkgState;

    //debug info 
    
    bool crc_ok             = false;
    bool crc_ok_header      = false;
    int  error_sum_payload  = 0;
    int  error_sum_header   = 0;
    int  read_sum           = 0;
    int  write_num          = 0;
    int  pkg_sum            = 0;
    int  state[5];
};
}
#endif  // SERIAL_DRIVER_MY_NODE_HPP_