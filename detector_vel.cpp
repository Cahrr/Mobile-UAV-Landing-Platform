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
#include <fstream>
using namespace std;

ros::Time c_time;
ros::Time l_time;
aruco_msgs::Detector detect;
aruco_msgs::Detector_vel detector_uav_vel_cmd;
geometry_msgs::Vector3 rpy;

// Arocu_ros 输出识别位置
double Xbody = 0.0, Ybody = 0.0, Zbody = 0.0;
double Xbody_ex = 0.0, Ybody_ex = 0.0, Zbody_ex = 0.0; //前一帧位置

float H = -0.55;
int flag = 0, id = -1, num_all = 0, num_right = 0;

double last_err_x = 0.0, last_err_y = 0.0, last_err_z = 0.0;
double current_time = 0.0;
double last_time = 0.0;

// 误差积分项
double err_sum_x = 0.0;
double err_sum_y = 0.0;
double err_sum_z = 0.0;

// 误差
double err_x = 0.0, err_x0 = 0.0;
double err_y = 0.0, err_y0 = 0.0;
double err_z = 0.0, err_z0 = 0.0;

// 原始PI
double P_x = 0.8, P_y = 0.8, P_z = 0.4;
double I_x = 0.007, I_y = 0.007;

// PID 增量式PID 模糊PID
double x_kp = 0.36, x_ki = 0.001, x_kd = 0.04;
double y_kp = 0.36, y_ki = 0.001, y_kd = 0.04;
double z_kp = 0.36, z_ki = 0.001, z_kd = 0.04;

// 增量式PID delta_vel
double x_icm_vel;
double y_icm_vel;
double z_icm_vel;

Incremental_PID x_icm_pid;
Incremental_PID y_icm_pid;
Incremental_PID z_icm_pid;

// 模糊PID
int fuzzy_flag = 0;
double dt = 0.0;
double derr_x = 0.0, derr_y = 0.0, derr_z = 0.0; // 误差的微分
double x_scale_err, y_scale_err, z_scale_err;    // 误差的量化因子
double x_scale_derr, y_scale_derr, z_scale_derr; // 误差微分的量化因子
double x_scale_kp, x_scale_ki, x_scale_kd;       // delta (kp ki kd)的量化因子
double y_scale_kp, y_scale_ki, y_scale_kd;
double z_scale_kp, z_scale_ki, z_scale_kd;
double x_fpid_output, y_fpid_output, z_fpid_output;    // 模糊PID的输出(增量)
double x_scale_output, y_scale_output, z_scale_output; // 模糊PID输出的量化因子

Incremental_PID x_fuzzy_icm_pid;
Incremental_PID y_fuzzy_icm_pid;
Incremental_PID z_fuzzy_icm_pid;

Incremental_PID x_fzy_pid;
Incremental_PID y_fzy_pid;
Incremental_PID z_fzy_pid;

Fuzzy_PID x_fuzzy_pid;
Fuzzy_PID y_fuzzy_pid;
Fuzzy_PID z_fuzzy_pid;

double *x_fzy_fpid;
double *y_fzy_fpid;
double *z_fzy_fpid;


/*-------------------------------------
data_record = 0  不记录数据
data_record = 1  记录数据
---------------------------------------*/
int data_record = 0;


/*-------------------------------------
control_method = 0   程序原始的PI控制
control_method = 1   PID
control_method = 2   增量 PID
control_method = 3   模糊 PID
---------------------------------------*/
int control_method = 3;


// 坐标转换
void detector_cb(const aruco_msgs::Detector::ConstPtr &msg)
{
    detect = *msg;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(detect.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;

    Xbody_ex = Xbody;
    Ybody_ex = Ybody;
    Zbody_ex = Zbody;

    Xbody = -detect.pose.position.y;
    Ybody = detect.pose.position.x;
    Zbody = -detect.pose.position.z;

    // 识别丢失情况
    if (Ybody == -1.0)
    {
        Xbody = Xbody_ex;
        Ybody = Ybody_ex;
        Zbody = Zbody_ex;
    }

    id = detect.id;
    num_all++;
}


int main(int argc, char **argv)
{
    // ROS订阅与发布
    ros::init(argc, argv, "detector_vel_node");
    ros::NodeHandle nh;
    ros::Subscriber detector_sub = nh.subscribe<aruco_msgs::Detector>("/aruco_single/detector", 25, detector_cb);
    ros::Publisher detector_vel_pub = nh.advertise<aruco_msgs::Detector_vel>("/aruco_single/detector_vel", 25);
    ros::Rate rate(25.0);

    // csv数据记录 
    if(data_record == 1)
    {
        ofstream oFile;
        oFile.open("/home/itr/pid_data/pid_data.csv", ios::out | ios::app); // 数据续写非覆盖
        oFile << "Ybody"
            << ","
            << "err_y"
            << ","
            << "err_sum_y"
            << ","
            << "derr_y"
            << ","
            << "vel_y"
            << ","
            << "y_icm_vel"
            << ","
            << "time" << endl; // endl 换行
    }

    while (ros::ok())
    {
        switch (control_method)
        {
        // PI
        case 0:
            // 位置识别限幅
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

            // 位置误差
            err_x = 0.0 - Xbody;
            err_y = 0.0 - Ybody;
            err_z = H - Zbody;

            // 位置误差积分
            err_sum_x += err_x;
            err_sum_y += err_y;
            err_sum_z += err_z;

            // 位置误差微分限幅
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

            // 满足marker在摄像头内条件，进入降落第一阶段
            if (id != -1)
                num_right++;
            if (num_all >= 30 && flag == 0 && num_right >= 25)
            { 
                flag = 1;
                num_all = 0;
                num_right = 0;
            }
            else if (num_all >= 30)
            {
                num_all = 0;
                num_right = 0;
            }

            // 顺利降落到相机坐标系下（0,0,0.3）左右，进入降落第二阶段
            if (flag == 1 && (-0.3) < err_x && err_x < 0.3 && (-0.3) < err_y && err_y < 0.3 && (-0.25) > Zbody && Zbody > (-1.1) && (id != -1))
            { 
                flag = 2;
                H = 0;
                err_sum_z = 0;
                ROS_INFO("case2");
            }

            ROS_INFO("case3      err_x:%f,err_y:%f,Zbody:%f,id:%d ", err_x, err_x, Zbody, id);
            
            // 顺利降落到相机坐标系下（0,0,0.1）左右，进入降落第三阶段
            if (flag == 2 && (-0.1) < err_x && err_x < 0.1 && (-0.1) < err_y && err_y < 0.1 && Zbody > (-0.2) && (id != -1))
            { 
                flag = 3;
                ROS_INFO("case3");
            }

            if (id != -1)
            {
                detector_uav_vel_cmd.vel.linear.x = P_x * err_x + I_x * err_sum_x;
                detector_uav_vel_cmd.vel.linear.y = P_y * err_y + I_y * err_sum_y;
                
                if (Zbody < (-0.3))
                {
                    detector_uav_vel_cmd.vel.linear.z = P_z * err_z;
                }
                else
                {
                    detector_uav_vel_cmd.vel.linear.z = 0.5 * err_z;
                }
                
                detector_uav_vel_cmd.flag = flag;
                detector_vel_pub.publish(detector_uav_vel_cmd);
            }
            else
            {
                detector_uav_vel_cmd.vel.linear.x = 0;
                detector_uav_vel_cmd.vel.linear.y = 0;
                detector_uav_vel_cmd.vel.linear.z = 0;
                detector_uav_vel_cmd.flag = 0; // 无人机锁桨降落
                detector_vel_pub.publish(detector_uav_vel_cmd);
            }
            break;

        //-----------------------------------------------------------------------------------------
        // PID
        case 1:

            // 时间微分
            c_time = ros::Time::now();
            current_time = c_time.toSec();
            dt = current_time - last_time;

            if (dt > 0.001)
            {
                // 误差e
                err_x = 0.0 - Xbody;
                err_y = 0.0 - Ybody;
                err_z = H - Zbody;

                err_sum_x += err_x;
                err_sum_y += err_y;
                err_sum_z += err_z;

                if (err_sum_y > 2)
                {
                    err_sum_y = 2;
                }
                if (err_sum_y < -2)
                {
                    err_sum_y = -2;
                }

                if (err_sum_x > 2)
                {
                    err_sum_x = 2;
                }
                if (err_sum_x < -2)
                {
                    err_sum_x = -2;
                }

                if (err_sum_z > 2)
                {
                    err_sum_z = 2;
                }
                if (err_sum_z < -2)
                {
                    err_sum_z = -2;
                }

                // 误差的微分ec
                derr_x = (err_x - last_err_x) / dt;
                derr_y = (err_y - last_err_y) / dt;
                derr_z = (err_z - last_err_z) / dt;
                detector_uav_vel_cmd.vel.linear.x = x_kp * err_x + x_ki * err_sum_x + x_kd * derr_x;
                detector_uav_vel_cmd.vel.linear.y = y_kp * err_y + y_ki * err_sum_y + y_kd * derr_y;
                detector_uav_vel_cmd.vel.linear.z = z_kp * err_z + z_ki * err_sum_z + z_kd * derr_z;
                detector_vel_pub.publish(detector_uav_vel_cmd);
            }
            last_err_x = err_x;
            last_err_y = err_y;
            last_err_z = err_z;
            last_time = current_time;
            break;

        //-----------------------------------------------------------------------------------------
        // 增量式PID
        case 2:

            x_icm_pid.init(0.0, x_kp, x_ki, x_kd);
            y_icm_pid.init(0.0, y_kp, y_ki, y_kd);
            z_icm_pid.init(H, z_kp, z_ki, z_kd);

            x_icm_vel = x_icm_pid.Incremental_PID_output(Xbody);
            y_icm_vel = y_icm_pid.Incremental_PID_output(Ybody);
            z_icm_vel = z_icm_pid.Incremental_PID_output(Zbody);

            detector_uav_vel_cmd.vel.linear.y += y_icm_vel;
            detector_uav_vel_cmd.vel.linear.x += x_icm_vel;
            detector_uav_vel_cmd.vel.linear.z += z_icm_vel;

            detector_vel_pub.publish(detector_uav_vel_cmd);
            ROS_INFO("x_vel:%f", detector_uav_vel_cmd.vel.linear.x);
            ROS_INFO("x_icm_vel:%f", x_icm_vel);
            ROS_INFO("y_vel:%f", detector_uav_vel_cmd.vel.linear.y);
            ROS_INFO("y_icm_vel:%f", y_icm_vel);
            ROS_INFO("z_vel:%f", detector_uav_vel_cmd.vel.linear.z);
            ROS_INFO("z_icm_vel:%f", z_icm_vel);
            break;

        //-----------------------------------------------------------------------------------------
        //  模糊PID
        case 3:

            c_time = ros::Time::now();
            current_time = c_time.toSec();
            dt = current_time - last_time;

            if (dt > 0.0001)
            {

                // 误差e(输入1)
                err_x = 0.0 - Xbody;
                err_y = 0.0 - Ybody;
                err_z = H - Zbody;

                // 误差的微分ec(输入2)
                derr_x = (err_x - last_err_x) / dt;
                derr_y = (err_y - last_err_y) / dt;
                derr_z = (err_z - last_err_z) / dt;

                // 赋PID初值
                if (fuzzy_flag == 0)
                {

                    ROS_INFO("fuzzy_flag changed!");
                    x_fuzzy_pid.init(0.0, x_kp, x_ki, x_kd);
                    y_fuzzy_pid.init(0.0, y_kp, y_ki, y_kd);
                    z_fuzzy_pid.init(H, z_kp, z_ki, z_kd);
                    fuzzy_flag = 1;
                }

                // delta(P I D)和两个输入的量化因子(需要调整)
                x_scale_kp = 0.036 / 3, x_scale_ki = 0.0002 / 3, x_scale_kd = 0.005 / 3, x_scale_err = 15, x_scale_derr = 15 * dt;
                y_scale_kp = 0.036 / 3, y_scale_ki = 0.0002 / 3, y_scale_kd = 0.005 / 3, y_scale_err = 15, y_scale_derr = 15 * dt;
                z_scale_kp = 0.05 / 3, z_scale_ki = 0.01 / 3, z_scale_kd = 0.01 / 3, z_scale_err = 10, z_scale_derr = 10 * dt;

                // x_fpid_output = x_fuzzy_pid.Fuzzy_PID_output(Xbody, err_x, derr_x, x_scale_kp, x_scale_ki, x_scale_kd, x_scale_err, x_scale_derr);
                // y_fpid_output = y_fuzzy_pid.Fuzzy_PID_output(Ybody, err_y, derr_y, y_scale_kp, y_scale_ki, y_scale_kd, y_scale_err, y_scale_derr, y_fuzzy_icm_pid);
                // z_fpid_output = z_fuzzy_pid.Fuzzy_PID_output(Zbody, err_z, derr_z, z_scale_kp, z_scale_ki, z_scale_kd, z_scale_err, z_scale_derr);

                x_fzy_fpid = x_fuzzy_pid.Fuzzy_PID_output(Xbody, err_x, derr_x, x_scale_kp, x_scale_ki, x_scale_kd, x_scale_err, x_scale_derr, x_fuzzy_icm_pid);        
                y_fzy_fpid = y_fuzzy_pid.Fuzzy_PID_output(Ybody, err_y, derr_y, y_scale_kp, y_scale_ki, y_scale_kd, y_scale_err, y_scale_derr, y_fuzzy_icm_pid);
                z_fzy_fpid = z_fuzzy_pid.Fuzzy_PID_output(Zbody, err_z, derr_z, z_scale_kp, z_scale_ki, z_scale_kd, z_scale_err, z_scale_derr, z_fuzzy_icm_pid);
                
                x_fzy_pid.init(0.0, x_fzy_fpid[0], x_fzy_fpid[1], x_fzy_fpid[2]);
                y_fzy_pid.init(0.0, y_fzy_fpid[0], y_fzy_fpid[1], y_fzy_fpid[2]);
                z_fzy_pid.init(H, z_fzy_fpid[0], z_fzy_fpid[1], z_fzy_fpid[2]);

                x_fpid_output = pid.Incremental_PID_output(Xbody);
                y_fpid_output = pid.Incremental_PID_output(Ybody);
                z_fpid_output = pid.Incremental_PID_output(Zbody);
                
                //输出的量化因子(需要调整)
                x_scale_output = 1.0;
                y_scale_output = 1.0;
                z_scale_output = 1.0;

                detector_uav_vel_cmd.vel.linear.x += x_fpid_output * x_scale_output;
                detector_uav_vel_cmd.vel.linear.y += y_fpid_output * y_scale_output;
                detector_uav_vel_cmd.vel.linear.z += z_fpid_output * z_scale_output;
                
                /*
                if(detector_uav_vel_cmd.vel.linear.y > 0.20)
                {
                    detector_uav_vel_cmd.vel.linear.y = 0.20;
                }

                if(detector_uav_vel_cmd.vel.linear.y < -0.20)
                {
                    detector_uav_vel_cmd.vel.linear.y = -0.20;
                }
                */
               
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
        ROS_INFO("*****************************************************");
        ROS_INFO("flag:%d", detector_uav_vel_cmd.flag);
        ROS_INFO("err_y:%f", err_y);

        ROS_INFO("derr_y:%f", derr_y);
        ROS_INFO("vel_y:%f", detector_uav_vel_cmd.vel.linear.y);
        ROS_INFO("dt:%f", dt);
        ROS_INFO("y_fpid_output:%f", y_fpid_output * y_scale_output);
        //ROS_INFO("fpid[0]:%f, fpid[1]:%f, fpid[2]:%f", fpid[0], fpid[1], fpid[2]);    
        ROS_INFO("*****************************************************");
        
        if(data_record == 1)
        {
            oFile << Ybody << "," << err_y << "," << err_sum_y << "," << derr_y << "," << detector_uav_vel_cmd.vel.linear.y << "," << y_icm_vel << "," << current_time << endl; // endl 换行
        }
        ros::spinOnce();
        rate.sleep();
    }
    oFile.close();
    return 0;
}
