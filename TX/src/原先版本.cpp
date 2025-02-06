#include <node.hpp>
#include <fstream>
#include <endian.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>


int Communication::serial_init(std::string name, int baud)
{
    try
    {
        ser.setPort(name);
        ser.setBaudrate(baud);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        ROS_ERROR_STREAM("Unable to open port");
        return -1;
    }

    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialied");
    }
    else
    {
        return -1;
    }
    return 1;
}

void Communication::vel_callback(const geometry_msgs::Twist &cmd_vel)
{
    if (__BYTE_ORDER == __LITTLE_ENDIAN)
    {
        Protocol_DownPackage_t packed;
        ROS_INFO("Receive a /cmd_vel message!");
        ROS_INFO("Linear Components:[%f,%f,%f]", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z);
        ROS_INFO("Angular Componets:[%f,%f,%f]", cmd_vel.angular.x, cmd_vel.angular.y, cmd_vel.angular.z);

        memset(&packed, 0, sizeof(packed));

        packed.data.chassis_move_vec.vx = -cmd_vel.linear.y*0.1;
        packed.data.chassis_move_vec.vy = cmd_vel.linear.x*0.1;
        packed.data.chassis_move_vec.wz = 0;

        ROS_INFO("v:[%f,%f,%f]", packed.data.chassis_move_vec.vx, packed.data.chassis_move_vec.vy , packed.data.chassis_move_vec.wz);


        packed.crc16 = crc16::CRC16_Calc(reinterpret_cast<uint8_t*>(&packed),sizeof(packed)-sizeof(uint16_t),UINT16_MAX);
        ROS_ERROR("[%d]",packed.crc16);

        ser.write(reinterpret_cast<uint8_t*>(&packed),sizeof(packed));
        ser.flushOutput();
    }
}

void Communication::Odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

// Protocol_ID_t Communication::receive_MCU(){
//     Protocol_UpPackageMCU_t mcu_;
    
//     size_t MCU_size = sizeof(Protocol_UpPackageMCU_t);

//     int a = ser.read(reinterpret_cast<uint8_t*>(&mcu_),sizeof(mcu_));   //read 这个函数，是把电控发来的数据接收到
//     // ROS_INFO("notice = %d",mcu_.data.notice );
//     return mcu_.data.notice;


// }

 void Communication::receive_MCU(){
    

    Protocol_ID_t id;
    Protocol_UpPackageMCU_t mcu_;
    Protocol_UpDataReferee_t ref_;
    ser.read(&id,sizeof(id));
    if(AI_ID_REF == id){
        ser.read(reinterpret_cast<uint8_t*>(&ref_),sizeof(ref_));
        if (crc16::CRC16_Verify(reinterpret_cast<uint8_t *>(&ref_), sizeof(ref_)))
        {
            std::memcpy(&ref_data, &(ref_), sizeof(ref_));
        }
        
    }
    else if (AI_ID_MCU == id){
        ser.read(reinterpret_cast<uint8_t*>(&mcu_),sizeof(mcu_));
        if(crc16::CRC16_Verify(reinterpret_cast<uint8_t*>(&mcu_),sizeof(mcu_))) 
        {
            std::memcpy(&mcu_data, &(mcu_), sizeof(mcu_));
            // ROS_INFO("notice: %d",this->mcu_data.data.notice);
            // std::cout<<this->mcu_data.data.notice<<std::endl;
            //std::cout<<this->mcu_data.data.ball_speed<<std::endl;
        }

    }
    // int a = ser.read(&mcu_,sizeof(mcu_));   //read 这个函数，是把电控发来的数据接收到

}




Communication::Communication()
{
    pitch, roll, yaw = 0.0f;

    ros::NodeHandle n1;
    ros::Subscriber write_odom = n1.subscribe("odom", 1000, &Communication::Odom_callback, this);
    ros::Subscriber write_vel = n1.subscribe("cmd_vel", 1000, &Communication::vel_callback, this);
    ros::Publisher read_pub = n1.advertise<std_msgs::String>("read", 10);
    ros::Publisher notice_pub = n1.advertise<std_msgs::UInt8>("notice", 10);
    if (serial_init("/dev/ttyACM0", 460800) == -1)
        return;
    ros::Rate loop_rate(200); // hz
    while (ros::ok())
    {

        ros::spinOnce();
        if (ser.available())
        {
            Protocol_ID_t notice1;
            receive_MCU();
            // ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String result;
            std_msgs::UInt8 notice_;
            notice_.data = mcu_data.data.notice;
            ROS_INFO("notice: %d",notice_.data);
            //notice_.data 
            //ROS_INFO("notice = %d",notice_.data);
            //result.data = ser.read(ser.available());
            // ROS_INFO_STREAM("Read:" << result.data);
            //read_pub.publish(result);
            notice_pub.publish(notice_);
        }
        loop_rate.sleep();
    }
}