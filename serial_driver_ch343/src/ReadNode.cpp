#include "serial_driver_ch343/ReadNode.hpp"


/**
 * TODO:
 *  do not want to work......
 * 
 *  1. can receive() and decode() in one thread ?
 *  1.1 if not, is it safe for anonther thread to run the decode()?
 *
 *  2. can decode() and publish in one thread ?
 *  2.1 if not, is it safe for anonther thread to run the publish?
 *  
 *  3. can all data transmit from ch343 to the destinantion node without error and drop ?
 */

namespace serial_driver
{

ReadNode::ReadNode(const rclcpp::NodeOptions & options) : rclcpp::Node("read", options)
, config(std::make_shared<SerialConfig>(2000000,8,false,StopBit::TWO,Parity::NONE)), port(std::make_shared<Port>(config))
{

    gimabal_msg_pub_ = this->create_publisher<msg_interfaces::msg::GimbalMsg>("/gimbal_msg", 10);
    sentry_gimbal_msg_pub_ = this->create_publisher<msg_interfaces::msg::SentryGimbalMsg>("/sentry_gimbal_msg", 10);
    //create the publisher
    // gimabal_msg_ = this->create_publisher<TwoCRC_GimbalMsg>("gimbal msg", 10);
    //open the port
    while(true)
    {
        if(!port->isPortOpen())
        {
            if(!port->openPort())
            {
                port->reopen();
            }
            else
            {
                break;
            }
        }
    }

    //receive 
    receive();
    //decode
    decode();
}

// void ReadNode::gimbalMsgCB(auto_aim_interfaces::msg::GimbalMsg::SharedPtr msg){

//     return;
// }

// void ReadNode::sentryGimbalMsgCB(auto_aim_interfaces::msg::SentryGimbalMsg::SharedPtr msg){
//     return;
// }

PkgState ReadNode::decode()
{
    int size = buffer.size();
    printf("size : %d \n",size);

    if( size < sizeof(Header) )
        return PkgState::HEADER_INCOMPLETE;

    for(int i = 0; i < size; i++)
    {
        if(buffer[i] == 0xAA)
        {
            std::copy(buffer.begin() + i, buffer.begin()+ i + sizeof(Header), decodeBuffer);
            crc_ok_header = crc16::Verify_CRC16_Check_Sum(decodeBuffer, sizeof(Header) );

            if( !crc_ok_header )
            {
                error_sum_header ++;
                buffer.erase(buffer.begin() + i, buffer.begin() + i + sizeof(Header));
                memset(decodeBuffer,0x00,sizeof(decodeBuffer));
                return PkgState::CRC_HEADER_ERRROR;
            }

            this->header = arrayToStruct<Header>(decodeBuffer);
            memset(decodeBuffer,0x00,sizeof(decodeBuffer));

            // pkg length = payload(dataLen) + header len (include header crc) + 2crc 
            if( i + (header.dataLen + sizeof(Header) + 2) > size )
            {
                bag_sum ++;
                return PkgState::PAYLOAD_INCOMPLETE;
            }

            printf("id: %d  \n",header.protocolID);
            std::copy(buffer.begin() + i ,buffer.begin() + i + header.dataLen + sizeof(Header) + 2, decodeBuffer);
            crc_ok = crc16::Verify_CRC16_Check_Sum(decodeBuffer,header.dataLen + sizeof(Header) + 2);
            printf("crc ok : %d \n",crc_ok);

            if(!crc_ok)
            {
                error_sum_payload ++;
                memset(decodeBuffer,0x00,sizeof(decodeBuffer));
                buffer.erase(buffer.begin(), buffer.begin() + i + header.dataLen + sizeof(Header) + 2);
                return PkgState::CRC_PKG_ERROR;
            }

            
            memset(decodeBuffer,0x00,sizeof(decodeBuffer));
            buffer.erase(buffer.begin(), buffer.begin() + i + header.dataLen + sizeof(Header) + 2);
            bag_sum ++;
            classify(decodeBuffer); 
            return PkgState::COMPLETE;
        }
    }
}

void ReadNode::classify(uint8_t* data)
{
    Header header = arrayToStruct<Header>(data);
    TwoCRC_GimbalMsg twoCRC_GimbalMsg;
    TwoCRC_SentryGimbalMsg twoCRC_SentryGimbalMsg;
    switch (header.protocolID)
    {
    case CommunicationType::TWOCRC_GIMBAL_MSG :
        twoCRC_GimbalMsg = arrayToStruct<TwoCRC_GimbalMsg>(data);
        // gimabal_msg_ -> publish(twoCRC_GimbalMsg);
        break;
    
    case CommunicationType::TWOCRC_SENTRY_GIMBAL_MSG :
        twoCRC_SentryGimbalMsg = arrayToStruct<TwoCRC_SentryGimbalMsg>(data);
        // sentry_gimbal_msg_ -> publish(twoCRC_SentryGimbalMsg);        
        break;
    default:
        break;
    }
}

int ReadNode::receive()
{
    int read_num = 0;
    read_num = port->receive(receiveBuffer);

    if(read_num > 0)
    {
        buffer.insert(buffer.end(),receiveBuffer,receiveBuffer + read_num);
    }
    else
    {
        RCLCPP_ERROR(get_logger(),"Can not receive correctly !");
        port->reopen();
        return read_num;
    }
}

}//serial_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(serial_driver::ReadNode)