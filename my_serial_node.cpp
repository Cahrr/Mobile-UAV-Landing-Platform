#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Char.h>
#include <std_msgs/Int16.h>
#include <aruco_msgs/Detector.h>
#include <aruco_msgs/Detector_vel.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>


#define	sBUFFERSIZE	7//send buffer size 串口发送缓存长度
uint8_t s_buffer[sBUFFERSIZE];//发送缓存
uint8_t sub_buffer[3];
char Input = false;
serial::Serial ros_ser;
aruco_msgs::Detector_vel detect_vel;

// 电机控制频率换算，串口发送
void detector_vel_cb(const aruco_msgs::Detector_vel::ConstPtr& msg)
{
    // 速度换算频率比例系数
    float k = 1;
    // 获取速度信息
    detect_vel = *msg;
    float vel = -detect_vel.vel.linear.y;
    if (vel > 0.08)
        {
            vel = 0.08;
        }
    if (vel < -0.08)
        {
            vel = -0.08;
        }
    // 整型移位处理
    vel = vel * k * 10000 + 10000;
    int error_vel = (int) vel;
    uint8_t error_vel_h = ((error_vel & 0xff00)>>8)&0x00ff;
    uint8_t error_vel_l = error_vel & 0x00ff;
    // 通信协议
    s_buffer[0] = 0x52;
    s_buffer[1] = 0x13;
    s_buffer[2] = error_vel_l;
    s_buffer[3] = error_vel_h;
    s_buffer[4] = 0x00;
    s_buffer[5] = 0x00;
    s_buffer[6] = 0x14;
    // 数据发送
    ros_ser.write(s_buffer,7);
    /*
    for (int i=0;i<9;i++)
    {
        sub_buffer[0] = s_buffer[i];
        sub_buffer[1] = s_buffer[i+1];
        sub_buffer[2] = s_buffer[i+2];
        ros_ser.write(sub_buffer,3);
	i = i+2;
        ROS_INFO_STREAM("send sub_buffer");
	
    }
    */
}

int main (int argc, char** argv)
    {
    ros::init(argc, argv, "my_serial_node");
    ros::NodeHandle n("~");
    //订阅主题command
    ros::Subscriber command_sub = n.subscribe<aruco_msgs::Detector_vel>("/aruco_single/detector_vel", 100, detector_vel_cb);
    //发布主题sensor
    ros::Publisher status_pub = n.advertise<std_msgs::String>("/status", 100);

    try
    {
        ros_ser.setPort("/dev/ttyUSB0");
        ros_ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ros_ser.setTimeout(to);
        ros_ser.open();
    }
    
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }
    
    if(ros_ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port opened");
    }
    else
    {
        return -1;
    }
    
    ros::Rate loop_rate(10000);

    while(ros::ok())
        {
        ros::spinOnce();
        if(ros_ser.available())
        {
            std_msgs::String serial_data;
            //获取串口数据
            serial_data.data = ros_ser.read(ros_ser.available());
            ROS_INFO_STREAM("Read: " << serial_data.data);
            //将串口数据发布到话题/status
            status_pub.publish(serial_data);
        }
        //loop_rate.sleep();
    }
}


