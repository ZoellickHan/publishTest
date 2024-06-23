#ifndef SERIAL_DRIVER_MY_NODE_HPP_
#define SERIAL_DRIVER_MY_NODE_HPP_

#define READER_BUFFER_SIZE 64
#define MAX_BUFFER_SIZE 2024
#define DECODE_BUFFER_SIZE 128

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

#include <chrono>
#include <cstring>
namespace serial_driver
{

class ReadNode : public rclcpp::Node
{
public:
  ReadNode(const rclcpp::NodeOptions & options);
  ~ReadNode();
  std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

private:    
    //function
    PkgState decode();
    int receive();
    void classify(uint8_t* data);

    std::shared_ptr<SerialConfig> config;
    //  (2000000,8,false,StopBit::TWO,Parity::NONE);

    std::shared_ptr<Port> port ;
    // std::make_shared<Port>(config);

    rclcpp::TimerBase::SharedPtr timer_;

    std::deque<uint8_t> buffer;
    uint8_t decodeBuffer[DECODE_BUFFER_SIZE];
    uint8_t receiveBuffer[READER_BUFFER_SIZE];

    //info
    rclcpp::Publisher<msg_interfaces::msg::GimbalMsg>::SharedPtr gimabal_msg_pub_;
    rclcpp::Publisher<msg_interfaces::msg::SentryGimbalMsg>::SharedPtr sentry_gimbal_msg_pub_;

    //thread?
    std::thread decode_thread;
    std::thread read_thread;
    std::mutex  deque_mutex;
    
    //protocol 
    Header header;
    PkgState pkgState;
    //debug info 
    int error_sum_header = 0;
    bool crc_ok          = false;
    bool crc_ok_header   = false;
    int  pkg_sum         = 0;
    int  error_sum_payload = 0;
    int read_sum         = 0;

    int state[5];
 
};

}
#endif  // SERIAL_DRIVER_MY_NODE_HPP_