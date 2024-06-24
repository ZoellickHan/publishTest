#include "serial_driver_ch343/ReadNode.hpp"

namespace serial_driver
{

ReadNode::ReadNode(const rclcpp::NodeOptions & options) : rclcpp::Node("read_ch343", options)
, config(std::make_shared<SerialConfig>(2000000,8,false,StopBit::TWO,Parity::NONE)), port(std::make_shared<Port>(config))
{
    RCLCPP_WARN(get_logger(),"Begin the Node read_ch343 !" );
    
    std::cout << "Creating ROS subscriptions...\n";

    gimbal_command_sub_ = this->create_subscription<msg_interfaces::msg::GimbalCommand>("/gimbal_command", rclcpp::SensorDataQoS(),
    std::bind(&ReadNode::GimbalCommand_CB,this,std::placeholders::_1));
    
    chassis_command_sub_ = this->create_subscription<msg_interfaces::msg::ChassisCommand>("/chassis_command", rclcpp::SensorDataQoS(),
    std::bind(&ReadNode::ChassisCommand_CB,this,std::placeholders::_1));

    sentry_gimbal_command_sub_ = this->create_subscription<msg_interfaces::msg::SentryGimbalCommand>("/sentry_gimbal_command", rclcpp::SensorDataQoS(),
    std::bind(&ReadNode::SentryGimbalCommand_CB,this,std::placeholders::_1));

    acction_sub_ = this->create_subscription<msg_interfaces::msg::TestMsg>("/test_msg", rclcpp::SensorDataQoS(),
    std::bind(&ReadNode::test_CB,this,std::placeholders::_1));

    test_2_sub_ = this->create_subscription<msg_interfaces::msg::TestMsg2>("/test_msg2", rclcpp::SensorDataQoS(),
    std::bind(&ReadNode::test2_CB,this,std::placeholders::_1));

    std::cout << "Creating ROS publisher...\n";

    gimabal_msg_pub_ = this->create_publisher<msg_interfaces::msg::GimbalMsg>("/gimbal_msg", 10);
    sentry_gimbal_msg_pub_ = this->create_publisher<msg_interfaces::msg::SentryGimbalMsg>("/sentry_gimbal_msg", 10);

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

    timer1_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&ReadNode::transmit,this));
    // timer2_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&ReadNode::rx,this));
    rx_thread = std::thread(&ReadNode::rx,this);
    // tx_thread = std::thread(&ReadNode::transmit,this);
}

void ReadNode::rx()
{
    while(true)
    { 
    receive();
    pkgState = PkgState::COMPLETE;
    while(receive_buffer.size() > 0 &&  pkgState != PkgState::HEADER_INCOMPLETE && pkgState != PkgState::PAYLOAD_INCOMPLETE)
    {
        pkgState = decode();
        static PkgState laststate = PkgState::COMPLETE;
        if(laststate == PkgState::OTHER  )
        {

        }
        else if ( pkgState == PkgState::OTHER)
        {
            printf("occur \n");
        }

        laststate = pkgState;

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

PkgState ReadNode::decode()
{    
    int size = receive_buffer.size(); 

    for(int i = 0; i < size; i++)
    {
        if(receive_buffer[i] == 0xAA)
        {
            if( i + sizeof(Header) > size)
            {
                printf("PkgState::HEADER_INCOMPLETE\n");
                return PkgState::HEADER_INCOMPLETE;
            }
            std::copy(receive_buffer.begin() + i, receive_buffer.begin()+ i + sizeof(Header), decodeBuffer);
            crc_ok_header = crc16::Verify_CRC16_Check_Sum(decodeBuffer, sizeof(Header));

            if( !crc_ok_header )
            {
                error_sum_header ++;
                try
                {
                    receive_buffer.erase(receive_buffer.begin() + i, receive_buffer.begin() + i + sizeof(Header));
                }catch(const std::exception & ex){
                    RCLCPP_ERROR(get_logger(), "Error creating serial port:  %s", ex.what());
                    printf("awful erase\n");
                }
                
                printf("PkgState::CRC_HEADER_ERRROR\n");
                return PkgState::CRC_HEADER_ERRROR;
            }
           
            this->header = arrayToStruct<Header>(decodeBuffer);
             
            // pkg length = payload(dataLen) + header len (include header crc) + 2crc 
            if( i + header.dataLen + sizeof(Header) + 2 > size )
            {
                printf("PkgState::PAYLOAD_INCOMPLETE\n");
                return PkgState::PAYLOAD_INCOMPLETE;
            }
            
            std::copy(receive_buffer.begin() + i ,receive_buffer.begin() + i + header.dataLen + sizeof(Header) + 2, decodeBuffer);
            crc_ok = crc16::Verify_CRC16_Check_Sum(decodeBuffer,header.dataLen + sizeof(Header) + 2);

            if(!crc_ok)
            {
                error_sum_payload ++;
                //payload error
                try{
                    //check if there is a coming pkg ?
                    for(int j = i + 1 ; j < header.dataLen + sizeof(Header) + 2 + i; j++)
                    {
                        if(receive_buffer[j] == 0xAA)
                        {                            
                           if( j + sizeof(Header) > header.dataLen + sizeof(Header) + 2 + i)
                           {
                                receive_buffer.erase(receive_buffer.begin(),receive_buffer.begin() + j);
                                return PkgState::HEADER_INCOMPLETE;
                           }

                            std::copy(receive_buffer.begin() + i, receive_buffer.begin()+ i + sizeof(Header), decodeBuffer);
                            crc_ok_header = crc16::Verify_CRC16_Check_Sum(decodeBuffer, sizeof(Header));

                            if(!crc_ok_header)
                            {
                                receive_buffer.erase(receive_buffer.begin(),receive_buffer.begin() + j + sizeof(Header));
                                j  += sizeof(Header) - 1;
                                continue;
                            }

                            this->header = arrayToStruct<Header>(decodeBuffer);

                            if( j + sizeof(Header) + header.dataLen + 2)
                            {
                                receive_buffer.erase(receive_buffer.begin(),receive_buffer.begin() + j);
                                return PkgState::PAYLOAD_INCOMPLETE;                                
                            }
                                
                            std::copy(receive_buffer.begin() + i ,receive_buffer.begin() + i + header.dataLen + sizeof(Header) + 2, decodeBuffer);
                            crc_ok = crc16::Verify_CRC16_Check_Sum(decodeBuffer,header.dataLen + sizeof(Header) + 2);

                            if(!crc_ok)
                            {
                                receive_buffer.erase(receive_buffer.begin(),receive_buffer.begin() + j + sizeof(Header) + header.dataLen +2 );
                                j  += sizeof(Header) + header.dataLen +2 - 1;
                                continue;
                            }
                        }                                                                                                                     
                    }
                    receive_buffer.erase(receive_buffer.begin(), receive_buffer.begin() + i + header.dataLen + sizeof(Header) + 2);
                }catch(const std::exception & ex){
                    RCLCPP_ERROR(get_logger(), "Error creating serial port:  %s", ex.what());
                    printf("awful 2\n");
                }
                
                printf("PkgState::CRC_PKG_ERROR\n");
                return PkgState::CRC_PKG_ERROR;
            }

            //complete
            try
            {
                receive_buffer.erase(receive_buffer.begin(), receive_buffer.begin() + i + header.dataLen + sizeof(Header) + 2);
            }catch(const std::exception & ex)
            {
                RCLCPP_ERROR(get_logger(), "Error creating serial port:  %s", ex.what());
                printf("awful 3 \n");
            }
            
            pkg_sum ++;

            std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> time = end - start;
           
            printf("crc error rate : %f pkg sum rate: %f ,read_sum reate: %f transmit hz :%f  time : %f \n",
            float(error_sum_header + error_sum_payload)/float(pkg_sum),double(pkg_sum)/time.count(),float(read_sum)*11/time.count(),trans_pkg_sum/time.count(),time.count());
            classify(decodeBuffer); 
            printf("PkgState::COMPLETE\n");
            return PkgState::COMPLETE;
        }
    
    }
    receive_buffer.erase(receive_buffer.begin(),receive_buffer.end());
    return PkgState::OTHER;
}

void ReadNode::classify(uint8_t* data)
{
    Header header = arrayToStruct<Header>(data);
    GimbalMsg twoCRC_GimbalMsg;
    SentryGimbalMsg twoCRC_SentryGimbalMsg;
    switch (header.protocolID)
    {
        case CommunicationType::TWOCRC_GIMBAL_MSG:
        {
            twoCRC_GimbalMsg = arrayToStruct<GimbalMsg>(data);
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
            twoCRC_SentryGimbalMsg = arrayToStruct<SentryGimbalMsg>(data);
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

    if(read_num > 0)
    {
        receive_buffer.insert(receive_buffer.end(),receiveBuffer,receiveBuffer + read_num);
    }
    else
    { 
        RCLCPP_ERROR(get_logger(),"Can not read from the ch343 !");
    }
    return read_num;
}

void ReadNode::eraser()
{
}

void ReadNode::test_CB(msg_interfaces::msg::TestMsg::SharedPtr msg)
{
    trans_pkg_sum += 2;
    RCLCPP_WARN(get_logger(),"Begin test Callback !!!!");
    ActionCommand twoCRC_ActionCommand;
    twoCRC_ActionCommand.header.protocolID = CommunicationType::TWOCRC_ACTION_CMD;
    twoCRC_ActionCommand.header.dataLen = sizeof(ActionCommand) - sizeof(Header) - 2;
    uint8_t buffer[sizeof(ActionCommand)];
    serial_driver::structToArray(twoCRC_ActionCommand,buffer);
    crc16::Append_CRC16_Check_Sum(buffer,sizeof(Header));
    crc16::Append_CRC16_Check_Sum(buffer,sizeof(ActionCommand));

    std::lock_guard<std::mutex> lockf(transmit_mutex);
    transmit_buffer.insert(transmit_buffer.end(),buffer, buffer + sizeof(ActionCommand));
    transmit_buffer.insert(transmit_buffer.end(),buffer, buffer + sizeof(ActionCommand));
}

void ReadNode::test2_CB(msg_interfaces::msg::TestMsg2::SharedPtr msg)
{
    trans_pkg_sum += 2;
    RCLCPP_WARN(get_logger(),"Begin test2 Callback !!!!");
    GimbalMsg  twoCRC_GimbalMsg ;
    uint8_t buffer[sizeof(GimbalMsg)];
    twoCRC_GimbalMsg.header.dataLen = sizeof(GimbalMsg) - sizeof(Header) -2;
    twoCRC_GimbalMsg.header.protocolID = CommunicationType::TWOCRC_GIMBAL_MSG;
    serial_driver::structToArray(twoCRC_GimbalMsg ,buffer);
    crc16::Append_CRC16_Check_Sum(buffer,sizeof(Header));
    crc16::Append_CRC16_Check_Sum(buffer,sizeof(GimbalMsg ));

    std::lock_guard<std::mutex> lockf(transmit_mutex);
    transmit_buffer.insert(transmit_buffer.end(),buffer, buffer + sizeof(GimbalMsg ));
    transmit_buffer.insert(transmit_buffer.end(),buffer, buffer + sizeof(GimbalMsg ));
}

void ReadNode::GimbalCommand_CB(msg_interfaces::msg::GimbalCommand::SharedPtr msg)
{
    trans_pkg_sum += 2;
    RCLCPP_WARN(get_logger(),"Begin GimbalCommand Callback !!!!");
    GimbalCommand twoCRC_GimbalCommand;
    uint8_t buffer[sizeof(GimbalCommand)];

    twoCRC_GimbalCommand.header.protocolID = CommunicationType::TWOCRC_GIMBAL_CMD;
    twoCRC_GimbalCommand.header.dataLen    = sizeof(GimbalCommand) - sizeof(Header) - 2;
    twoCRC_GimbalCommand.shoot_mode        = msg->shoot_mode;
    twoCRC_GimbalCommand.target_pitch      = msg->target_pitch;
    twoCRC_GimbalCommand.target_yaw        = msg->target_yaw;

    serial_driver::structToArray(twoCRC_GimbalCommand,buffer);
    crc16::Append_CRC16_Check_Sum(buffer,sizeof(Header));
    crc16::Append_CRC16_Check_Sum(buffer,sizeof(GimbalCommand));

    std::lock_guard<std::mutex> lockf(transmit_mutex);
    transmit_buffer.insert(transmit_buffer.end(),buffer, buffer + sizeof(GimbalCommand));
    transmit_buffer.insert(transmit_buffer.end(),buffer, buffer + sizeof(GimbalCommand));
}

void ReadNode::ChassisCommand_CB(msg_interfaces::msg::ChassisCommand::SharedPtr msg)
{
    trans_pkg_sum += 2;
    RCLCPP_WARN(get_logger(),"Begin ChassisCommand Callback");
    ChassisCommand twoCRC_ChassisCommand;
    uint8_t buffer[sizeof(ChassisCommand)];

    twoCRC_ChassisCommand.header.protocolID = serial_driver::CommunicationType::TWOCRC_CHASSIS_CMD;
    twoCRC_ChassisCommand.header.dataLen    = sizeof(ChassisCommand) - sizeof(Header) - 2;
    twoCRC_ChassisCommand.vel_w             = msg->vel_w;
    twoCRC_ChassisCommand.vel_x             = msg->vel_x;
    twoCRC_ChassisCommand.vel_y             = msg->vel_y;

    serial_driver::structToArray(twoCRC_ChassisCommand,buffer);
    crc16::Append_CRC16_Check_Sum(buffer,sizeof(Header));
    crc16::Append_CRC16_Check_Sum(buffer,sizeof(ChassisCommand));

    std::lock_guard<std::mutex> lockf(transmit_mutex);
    transmit_buffer.insert(transmit_buffer.end(),buffer,buffer+sizeof(ChassisCommand));
    transmit_buffer.insert(transmit_buffer.end(),buffer,buffer+sizeof(ChassisCommand));
}

void ReadNode::SentryGimbalCommand_CB(msg_interfaces::msg::SentryGimbalCommand::SharedPtr msg)
{
    trans_pkg_sum += 2;
    RCLCPP_WARN(get_logger(),"Begin SentryGimbalCommand Callback");

    SentryGimbalCommand twoCRC_SentryGimbalCommand;
    uint8_t buffer[sizeof(SentryGimbalCommand)];

    twoCRC_SentryGimbalCommand.header.protocolID = serial_driver::CommunicationType::TWOCRC_SENTRY_GIMBAL_CMD;
    twoCRC_SentryGimbalCommand.header.dataLen    = sizeof(SentryGimbalCommand) - sizeof(Header) - 2;
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
    crc16::Append_CRC16_Check_Sum(buffer,sizeof(SentryGimbalCommand));

    std::lock_guard<std::mutex> lockf(transmit_mutex);
    transmit_buffer.insert(transmit_buffer.end(),buffer,buffer+sizeof(SentryGimbalCommand));
    transmit_buffer.insert(transmit_buffer.end(),buffer,buffer+sizeof(SentryGimbalCommand));
}

int ReadNode::transmit()
{
    uint8_t buffer[TRANSMIT_BUFFER];
    long size = transmit_buffer.size();
    // printf("transmitsize : %d \n",size);
    // printf("trans_pkg:%d \n",trans_pkg_sum);
    if(size > TRANSMIT_BUFFER)
    {
        while(size > 2*TRANSMIT_BUFFER && transmit_buffer.size() > 0)
        {
            size -= TRANSMIT_BUFFER;
            // printf("size : %ld \n",size);
            std::lock_guard<std::mutex> lockf(transmit_mutex);
            std::copy(transmit_buffer.begin(),transmit_buffer.begin()+TRANSMIT_BUFFER,buffer);
            transmit_buffer.erase(transmit_buffer.begin(),transmit_buffer.begin() + TRANSMIT_BUFFER);
            // std::lock_guard<std::mutex> unlock
            write_num = write(port->fd,buffer,TRANSMIT_BUFFER);

            if(write_num < 0)
                RCLCPP_ERROR(get_logger(),"Can not transmit");
        }
    }
    
}

}//serial_driver
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(serial_driver::ReadNode)