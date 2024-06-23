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

ReadNode::ReadNode(const rclcpp::NodeOptions & options) : rclcpp::Node("read_ch343", options)
, config(std::make_shared<SerialConfig>(2000000,8,false,StopBit::TWO,Parity::NONE)), port(std::make_shared<Port>(config))
{
    RCLCPP_WARN(get_logger(),"Begin the Node read_ch343 !" );

    gimabal_msg_pub_ = this->create_publisher<msg_interfaces::msg::GimbalMsg>("/gimbal_msg", 10);

    sentry_gimbal_msg_pub_ = this->create_publisher<msg_interfaces::msg::SentryGimbalMsg>("/sentry_gimbal_msg", 10);

    gimbal_command_sub_ = this->create_subscription<msg_interfaces::msg::GimbalCommand>("/gimbal_commmand",rclcpp::SensorDataQoS(),
    std::bind(&ReadNode::GimbalCommand_CB,this,std::placeholders::_1));

    chassis_command_sub_ = this->create_subscription<msg_interfaces::msg::ChassisCommand>("/chassis_command",rclcpp::SensorDataQoS(),
    std::bind(&ReadNode::ChassisCommand_CB,this,std::placeholders::_1));

    sentry_gimbal_command_sub_ = this->create_subscription<msg_interfaces::msg::SentryGimbalCommand>("/sentry_gimbal_command",rclcpp::SensorDataQoS(),
    std::bind(&ReadNode::SentryGimbalCommand_CB,this,std::placeholders::_1));

    //open the port
    while(true)
    {
      if(port->isPortOpen())
      {
        break;
      } 
      else
      {
        port->openPort();
      }
    }

    tx_thread = std::thread(&ReadNode::transmit,this);

    while(true)
    {
        receive();

        pkgState = PkgState::COMPLETE;
        while(receive_buffer.size() > 0 &&  pkgState != PkgState::HEADER_INCOMPLETE && pkgState != PkgState::PAYLOAD_INCOMPLETE)
        {
            pkgState = decode();
            switch (pkgState)
            {
            case PkgState::COMPLETE :
                    state[0] ++ ;
                    break;
            
            case PkgState::CRC_HEADER_ERRROR :
                    state[3] ++ ;
                    break;

                case PkgState::CRC_PKG_ERROR :
                    state[4] ++;
                    break;

                case PkgState::HEADER_INCOMPLETE :
                    state[1] ++;
                    break;

                case PkgState::PAYLOAD_INCOMPLETE :
                    state[2] ++;
                    break;

            default:
                break;
            }
        }  
    }
}

ReadNode::~ReadNode(){
    throw std::invalid_argument( "received negative value" );
}

PkgState ReadNode::decode()
{    
    int size = receive_buffer.size();
    if( size < sizeof(Header) )
        return PkgState::HEADER_INCOMPLETE;

    for(int i = 0; i < size; i++)
    {
        if(receive_buffer[i] == 0xAA)
        {
            std::copy(receive_buffer.begin() + i, receive_buffer.begin()+ i + sizeof(Header), decodeBuffer);
            crc_ok_header = crc16::Verify_CRC16_Check_Sum(decodeBuffer, sizeof(Header));

            if( !crc_ok_header )
            {
                error_sum_header ++;
                receive_buffer.erase(receive_buffer.begin() + i, receive_buffer.begin() + i + sizeof(Header));
                return PkgState::CRC_HEADER_ERRROR;
            }

            this->header = arrayToStruct<Header>(decodeBuffer);

            // pkg length = payload(dataLen) + header len (include header crc) + 2crc 
            if( i + (header.dataLen + sizeof(Header) + 2) > size )
            {
                // pkg_sum ++;
                return PkgState::PAYLOAD_INCOMPLETE;
            }

            std::copy(receive_buffer.begin() + i ,receive_buffer.begin() + i + header.dataLen + sizeof(Header) + 2, decodeBuffer);
            crc_ok = crc16::Verify_CRC16_Check_Sum(decodeBuffer,header.dataLen + sizeof(Header) + 2);

            if(!crc_ok)
            {
                error_sum_payload ++;
                receive_buffer.erase(receive_buffer.begin(), receive_buffer.begin() + i + header.dataLen + sizeof(Header) + 2);
                return PkgState::CRC_PKG_ERROR;
            }
            
            receive_buffer.erase(receive_buffer.begin(), receive_buffer.begin() + i + header.dataLen + sizeof(Header) + 2);
            pkg_sum ++;

            std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> time = end - start;
           
            // printf("crc error rate : %f pkg sum rate: %f ,read_sum reate: %f time : %f \n",
            // float(error_sum_header + error_sum_payload)/float(pkg_sum),double(pkg_sum)/time.count(),float(read_sum)*11/time.count(),time.count());
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
        case CommunicationType::TWOCRC_GIMBAL_MSG:
        {
            twoCRC_GimbalMsg = arrayToStruct<TwoCRC_GimbalMsg>(data);
            msg_interfaces::msg::GimbalMsg gimbalmsg;
            gimbalmsg.bullet_speed = twoCRC_GimbalMsg.bullet_speed;
            gimbalmsg.cur_cv_mode  = twoCRC_GimbalMsg.cur_cv_mode;
            gimbalmsg.q_w          = twoCRC_GimbalMsg.q_w;

            gimbalmsg.q_x          = twoCRC_GimbalMsg.q_x;
            gimbalmsg.q_y          = twoCRC_GimbalMsg.q_y;
            gimbalmsg.q_z          = twoCRC_GimbalMsg.q_z;
            gimabal_msg_pub_ -> publish(gimbalmsg);
            break;
        }
        case CommunicationType::TWOCRC_SENTRY_GIMBAL_MSG:
        {
            twoCRC_SentryGimbalMsg = arrayToStruct<TwoCRC_SentryGimbalMsg>(data);
            msg_interfaces::msg::SentryGimbalMsg sentryGimbalMsg;
            sentryGimbalMsg.bullet_speed    = twoCRC_SentryGimbalMsg.bullet_speed;
            sentryGimbalMsg.big_q_w         = twoCRC_SentryGimbalMsg.big_q_w;
            sentryGimbalMsg.big_q_x         = twoCRC_SentryGimbalMsg.big_q_x; 
            sentryGimbalMsg.big_q_y         = twoCRC_SentryGimbalMsg.big_q_y;
            sentryGimbalMsg.big_q_z         = twoCRC_SentryGimbalMsg.big_q_z; 
            sentryGimbalMsg.cur_cv_mode     = twoCRC_SentryGimbalMsg.cur_cv_mode; 
            sentryGimbalMsg.small_q_w       = twoCRC_SentryGimbalMsg.small_q_w; 
            sentryGimbalMsg.small_q_x       = twoCRC_SentryGimbalMsg.small_q_x; 
            sentryGimbalMsg.small_q_y       = twoCRC_SentryGimbalMsg.small_q_y;
            sentryGimbalMsg.small_q_z       = twoCRC_SentryGimbalMsg.small_q_z;
            sentryGimbalMsg.target_color    = twoCRC_SentryGimbalMsg.target_color;

            sentry_gimbal_msg_pub_ -> publish(sentryGimbalMsg);        
            break;
        }
        default:
        {
            printf("type is not defined !\n");
            break;
        }
    }
}

int ReadNode::receive()
{
    int read_num = 0;
    read_num = read(port->fd,receiveBuffer,64);
    read_sum += read_num;

    if(read_num >= 0)
    {
        receive_buffer.insert(receive_buffer.end(),receiveBuffer,receiveBuffer + read_num);
    }
    else
    { 
        RCLCPP_ERROR(get_logger(),"Can not read from the ch343 !");
    }
    return read_num;
}

void ReadNode::GimbalCommand_CB(msg_interfaces::msg::GimbalCommand::SharedPtr msg)
{
    RCLCPP_WARN(get_logger(),"Begin GimbalCommand Callback");
    TwoCRC_GimbalCommand twoCRC_GimbalCommand;
    uint8_t buffer[sizeof(TwoCRC_GimbalCommand)];

    twoCRC_GimbalCommand.header.protocolID = CommunicationType::TWOCRC_GIMBAL_CMD;
    twoCRC_GimbalCommand.header.dataLen    = sizeof(TwoCRC_GimbalCommand) - sizeof(Header) - 2;
    twoCRC_GimbalCommand.shoot_mode        = msg->shoot_mode;
    twoCRC_GimbalCommand.target_pitch      = msg->target_pitch;
    twoCRC_GimbalCommand.target_yaw        = msg->target_yaw;

    serial_driver::structToArray(twoCRC_GimbalCommand,buffer);
    crc16::Append_CRC16_Check_Sum(buffer,sizeof(Header));
    crc16::Append_CRC16_Check_Sum(buffer,sizeof(TwoCRC_GimbalCommand));

    std::lock_guard<std::mutex> lockf(transmit_mutex);
    transmit_buffer.insert(transmit_buffer.end(),buffer, buffer + sizeof(TwoCRC_GimbalCommand));

}

void ReadNode::ChassisCommand_CB(msg_interfaces::msg::ChassisCommand::SharedPtr msg)
{
    RCLCPP_WARN(get_logger(),"Begin ChassisCommand Callback");
    TwoCRC_ChassisCommand twoCRC_ChassisCommand;
    uint8_t buffer[sizeof(TwoCRC_ChassisCommand)];

    twoCRC_ChassisCommand.header.protocolID = serial_driver::CommunicationType::TWOCRC_CHASSIS_CMD;
    twoCRC_ChassisCommand.header.dataLen    = sizeof(TwoCRC_ChassisCommand) - sizeof(Header) - 2;
    twoCRC_ChassisCommand.vel_w             = msg->vel_w;
    twoCRC_ChassisCommand.vel_x             = msg->vel_x;
    twoCRC_ChassisCommand.vel_y             = msg->vel_y;

    serial_driver::structToArray(twoCRC_ChassisCommand,buffer);
    crc16::Append_CRC16_Check_Sum(buffer,sizeof(Header));
    crc16::Append_CRC16_Check_Sum(buffer,sizeof(TwoCRC_ChassisCommand));

    std::lock_guard<std::mutex> lockf(transmit_mutex);
    transmit_buffer.insert(transmit_buffer.end(),buffer,buffer+sizeof(TwoCRC_ChassisCommand));
}

void ReadNode::SentryGimbalCommand_CB(msg_interfaces::msg::SentryGimbalCommand::SharedPtr msg)
{
    RCLCPP_WARN(get_logger(),"Begin SentryGimbalCommand Callback");

    TwoCRC_SentryGimbalCommand twoCRC_SentryGimbalCommand;
    uint8_t buffer[sizeof(TwoCRC_SentryGimbalCommand)];

    twoCRC_SentryGimbalCommand.header.protocolID = serial_driver::CommunicationType::TWOCRC_SENTRY_GIMBAL_CMD;
    twoCRC_SentryGimbalCommand.header.dataLen    = sizeof(TwoCRC_SentryGimbalCommand) - sizeof(Header) - 2;
    twoCRC_SentryGimbalCommand.l_shoot_mode      = msg->l_shoot_mode;
    twoCRC_SentryGimbalCommand.l_target_pitch    = msg->l_target_pitch;
    twoCRC_SentryGimbalCommand.l_target_yaw      = msg->l_target_yaw;
    twoCRC_SentryGimbalCommand.r_shoot_mode      = msg->r_shoot_mode;
    twoCRC_SentryGimbalCommand.r_target_pitch    = msg->r_target_pitch;
    twoCRC_SentryGimbalCommand.r_target_yaw      = msg->r_target_yaw;
    twoCRC_SentryGimbalCommand.main_target_pitch = msg->main_target_pitch;
    twoCRC_SentryGimbalCommand.main_target_yaw   = msg->main_target_yaw;

    serial_driver::structToArray(twoCRC_SentryGimbalCommand,buffer);
    crc16::Append_CRC16_Check_Sum(buffer,sizeof(Header));
    crc16::Append_CRC16_Check_Sum(buffer,sizeof(TwoCRC_SentryGimbalCommand));

    std::lock_guard<std::mutex> lockf(transmit_mutex);
    transmit_buffer.insert(transmit_buffer.end(),buffer,buffer+sizeof(TwoCRC_SentryGimbalCommand));
}

int ReadNode::transmit()
{
    uint8_t buffer[TRANSMIT_BUFFER];
    while(true)
    {
        int size = transmit_buffer.size();
        printf("transmitsize : %d \n",size);
        if(size > TRANSMIT_BUFFER)
        {
            for(int i = size; i < 2*TRANSMIT_BUFFER; i -= TRANSMIT_BUFFER)
            {
                std::copy(transmit_buffer.begin(),transmit_buffer.begin()+TRANSMIT_BUFFER,buffer);
                transmit_buffer.erase(transmit_buffer.begin(),transmit_buffer.begin() + TRANSMIT_BUFFER);
                write_num = write(port->fd,buffer,TRANSMIT_BUFFER);
                if(write_num < 0)
                    RCLCPP_ERROR(get_logger(),"Can not transmit");
            }
        }
    }
}

}//serial_driver
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(serial_driver::ReadNode)