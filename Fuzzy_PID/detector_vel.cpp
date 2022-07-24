/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */
#include <iostream>
#include <ros/ros.h>
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

#include "Fuzzy_PID.h"

aruco_msgs::Detector detect;
aruco_msgs::Detector_vel detector_uav_vel_cmd;
double Xbody = 0.0, Ybody = 0.0, Zbody = 0.0;

//误差积分项
double err_sum_x = 0.0;
double err_sum_y = 0.0;
double err_sum_z = 0.0;

//误差
double err_x = 0.0, err_x0 = 0.0;
double err_y = 0.0, err_y0 = 0.0;
double err_z = 0.0, err_z0 = 0.0;

double P_x = 0.8, P_y = 0.8,·····P_z = 0.4;
// double I_x=0.007,I_y=0.008
double I_x = 0.007, I_y = 0.007;

float H = -0.55;
int flag = 0, id = -1, num_all = 0, num_right = 0;

int i;

// control_method = 0   程序原始的PI控制
// control_method = 1   PID
// control_method = 2   模糊PID
int control_method = 0;

double last_err_x = 0.0, last_err_y = 0.0, last_err_z = 0.0;
double current_time = 0.0, last_time = 0.0;
double dt = 0.0;
double derr_x = 0.0, derr_y = 0.0, derr_z = 0.0; // 误差的微分
double x_scale_err, y_scale_err, z_scale_err;    // 误差的量化因子
double x_scale_derr, y_scale_derr, z_scale_derr; // 误差微分的量化因子
double x_scale_kp, x_scale_ki, x_scale_kd;       // delta (kp ki kd)的量化因子
double y_scale_kp, y_scale_ki, y_scale_kd;
double z_scale_kp, z_scale_ki, z_scale_kd;
double x_fpid_output, y_fpid_output, z_fpid_output;    // 模糊PID的输出(增量)
double x_scale_output, y_scale_output, z_scale_output; // 模糊PID输出的量化因子

geometry_msgs::Vector3 rpy;

//坐标转换
void detector_cb(const aruco_msgs::Detector::ConstPtr &msg)
{
    detect = *msg;
    // 将接收到的消息打印出来
    //  ROS_INFO("x:%f  y:%f  z:%f", Xbody, Ybody, Zbody);
    tf::Quaternion quat;
    tf::quaternionMsgToTF(detect.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    // the found angles are written in a geometry_msgs::Vector3.
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;
    Xbody = -detect.pose.position.y;
    Ybody = detect.pose.position.x;
    Zbody = -detect.pose.position.z;
    id = detect.id;
    num_all++;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detector_vel_node");
    ros::NodeHandle nh;

    ros::Subscriber detector_sub = nh.subscribe<aruco_msgs::Detector>("/aruco_single/detector", 25, detector_cb);
    ros::Publisher detector_vel_pub = nh.advertise<aruco_msgs::Detector_vel>("/aruco_single/detector_vel", 25);
    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(25.0);
    // wait for FCU connection
    /*    do{
            rate.sleep();
            ros::spinOnce();
            ROS_INFO("sleep");
        }while(Zbody>-1.35);*/
    /*for(i=0;i<300;i++){
        rate.sleep();
    }*/
    while (ros::ok())
    {
        switch (control_method)
        {
        case 0:
            if (Xbody > 1.0)
            {
                Xbody = 0.5;
            }
            if (Xbody < -1.0)
            {
                Xbody = -0.5;
            }
            if (Ybody > 1.0)
            {
                Ybody = 0.5;
            }
            if (Ybody < -1.0)
            {
                Ybody = -0.5;
            }

            err_x = 0.0 - Xbody;
            err_y = 0.0 - Ybody;
            err_z = H - Zbody;

            err_sum_x += err_x;
            err_sum_y += err_y;
            err_sum_z += err_z;

            if (err_sum_x > 5)
            {
                err_sum_x = 5;
            }
            if (err_sum_x < -5)
            {
                err_sum_x = -5;
            }
            if (err_sum_y > 2)
            {
                err_sum_y = 2;
            }
            if (err_sum_y < -2)
            {
                err_sum_y = -2;
            }
            if (err_sum_z > 5)
            {
                err_sum_z = 5;
            }
            if (err_sum_z < -5)
            {
                err_sum_z = -5;
            }
            if (id != -1)
                num_right++;
            if (num_all >= 30 && flag == 0 && num_right >= 25)
            { //满足mark在摄像头内条件，进入降落第一阶段
                flag = 1;
                num_all = 0;
                num_right = 0;
            }
            else if (num_all >= 30)
            {
                num_all = 0;
                num_right = 0;
            }
            //        ROS_INFO("case2      err_x:%f,err_y:%f,Zbody:%f", err_x,err_x,Zbody);
            if (flag == 1 && (-0.3) < err_x && err_x < 0.3 && (-0.3) < err_y && err_y < 0.3 && (-0.25) > Zbody && Zbody > (-1.1) && (id != -1))
            { //顺利降落到相机坐标系下（0，0，1）左右，进入第二阶段
                flag = 2;
                H = 0;
                err_sum_z = 0;
                ROS_INFO("case2");
            }
            ROS_INFO("case3      err_x:%f,err_y:%f,Zbody:%f,id:%d ", err_x, err_x, Zbody, id);
            if (flag == 2 && (-0.1) < err_x && err_x < 0.1 && (-0.1) < err_y && err_y < 0.1 && Zbody > (-0.2) && (id != -1))
            { //顺利降落到相机坐标系下（0，0，0.3）左右，进入第三阶段
                flag = 3;
                ROS_INFO("case3");
            }
            if (id != -1)
            {
                detector_uav_vel_cmd.vel.linear.x = P_x * err_x + I_x * err_sum_x;
                detector_uav_vel_cmd.vel.linear.y = P_y * err_y + I_y * err_sum_y;
                // detector_uav_vel_cmd.vel.linear.x = P_x * err_x;
                // detector_uav_vel_cmd.vel.linear.y = P_y * err_y;
                if (Zbody < (-0.3))
                {
                    detector_uav_vel_cmd.vel.linear.z = P_z * err_z;
                }
                else
                {
                    detector_uav_vel_cmd.vel.linear.z = 0.5 * err_z;
                }
                //            detector_uav_vel_cmd.vel.angular.z = -0.2 * rpy.z;
                detector_uav_vel_cmd.flag = flag;
                detector_vel_pub.publish(detector_uav_vel_cmd);
            }
            else
            {
                detector_uav_vel_cmd.vel.linear.x = 0;
                detector_uav_vel_cmd.vel.linear.y = 0;
                detector_uav_vel_cmd.vel.linear.z = 0;
                //            detector_uav_vel_cmd.vel.angular.z = 0;
                detector_uav_vel_cmd.flag = 0;
                detector_vel_pub.publish(detector_uav_vel_cmd);
            }
            break;

        // PID
        case 1:

            break;

        //模糊PID
        case 2:
            current_time = ros::time::now();
            dt = current_time - last_time;

            if (dt > 0.001)
            {
                //误差e(输入1)
                err_x = 0.0 - Xbody;
                err_y = 0.0 - Ybody;
                err_z = H - Zbody;

                //误差的微分ec(输入2)
                derr_x = (err_x - last_err_x) / dt;
                derr_y = (err_y - last_err_y) / dt;
                derr_z = (err_z - last_err_z) / dt;

                //初始的PID
                double x_kp = 0.5, x_ki = 0.05, x_kd = 0.1;
                double y_kp = 0.5, y_ki = 0.05, y_kd = 0.1;
                double z_kp = 0.5, z_ki = 0.05, z_kd = 0.1;

                Fuzzy_PID x_fuzzy_pid;
                Fuzzy_PID y_fuzzy_pid;
                Fuzzy_PID z_fuzzy_pid;

                x_fuzzy_pid.init(0.0, x_kp, x_ki, x_kd);
                y_fuzzy_pid.init(0.0, y_kp, y_ki, y_kd);
                z_fuzzy_pid.init(double(H), z_kp, z_ki, z_kd);

                //delta(P I D)和两个输入的量化因子(需要调整)
                x_scale_kp = 0.05 / 3, x_scale_ki = 0.01 / 3, x_scale_kd = 0.01 / 3, x_scale_err = 10, x_scale_derr = 10 * dt;
                y_scale_kp = 0.05 / 3, y_scale_ki = 0.01 / 3, y_scale_kd = 0.01 / 3, y_scale_err = 10, y_scale_derr = 10 * dt;
                z_scale_kp = 0.05 / 3, z_scale_ki = 0.01 / 3, z_scale_kd = 0.01 / 3, z_scale_err = 10, z_scale_derr = 10 * dt;

                x_fpid_output = x_fuzzy_pid.Fuzzy_PID_output(Xbody, err_x, derr_x, x_scale_kp, x_scale_ki, x_scale_kd, x_scale_err, x_scale_derr);
                y_fpid_output = y_fuzzy_pid.Fuzzy_PID_output(Ybody, err_y, derr_y, y_scale_kp, y_scale_ki, y_scale_kd, y_scale_err, y_scale_derr);
                z_fpid_output = z_fuzzy_pid.Fuzzy_PID_output(Zbody, err_z, derr_z, z_scale_kp, z_scale_ki, z_scale_kd, z_scale_err, z_scale_derr);

                //输出的量化因子(需要调整)
                x_scale_output = 1.0;
                y_scale_output = 1.0;
                z_scale_output = 1.0;

                detector_uav_vel_cmd.vel.linear.x += x_fpid_output * x_scale_output;
                detector_uav_vel_cmd.vel.linear.y += y_fpid_output * y_scale_output;
                detector_uav_vel_cmd.vel.linear.z += z_fpid_output * z_scale_output;

                detector_vel_pub.publish(detector_uav_vel_cmd);
            }
            last_time = current_time;
            last_err_x = err_x;
            last_err_y = err_y;
            last_err_z = err_z;

            break;

        default:
            break;
        }
        
        ROS_INFO("flag:%d", detector_uav_vel_cmd.flag);
        ROS_INFO("x:%f  y:%f  z:%f", Xbody, Ybody, Zbody);
        ROS_INFO("x_vel:%f  y_vel:%f  z_vel:%f", detector_uav_vel_cmd.vel.linear.x, detector_uav_vel_cmd.vel.linear.y, detector_uav_vel_cmd.vel.linear.z);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
