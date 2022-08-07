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
#include "Data.cpp"
#include <fstream>
using namespace std;

ros::Time c_time;
ros::Time l_time;
aruco_msgs::Detector detect;
aruco_msgs::Detector_vel detector_uav_vel_cmd;
geometry_msgs::Vector3 rpy;

// 跟随目标高度
float H = -0.6;
// 降落阶段判断时间
int land_time1 = 0, land_time2 = 0;
// Aruco识别标志位
int id = -1;
// 控制状态标志位
int flag = 0;
// 降落状态标志位
status = 0
 num_all = 0, num_right = 0, land = 0, , 
double current_time = 0.0, last_time = 0.0;

// 识别位置
Body body = {0.0, 0.0, 0.0};
// 前一帧识别位置
Body last_body = {0.0, 0.0, 0.0};
// 位置误差
Err err = {0.0, 0.0, 0.0};
// 前一帧位置误差
Err last_err = {0.0, 0.0, 0.0};
// 位置误差积分
Err_sum err_sum = {0.0, 0.0, 0.0};
// 位置误差微分
dErr derr = {0.0, 0.0, 0.0};
// 限幅
Range range;
// PID控制参数
PID pid = {range, body, err, err_sum, derr};

// 原始PI
double P_x = 0.8, P_y = 0.8, P_z = 0.4;
double I_x = 0.007, I_y = 0.007;

// PID 增量式PID 模糊PID
double x_kp = 0.36, x_ki = 0.001, x_kd = 0.04;
double y_kp = 0.36, y_ki = 0.002, y_kd = 0.04;
double z_kp = 0.36, z_ki = 0.001, z_kd = 0.01;

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
double derr.x = 0.0, derr.y = 0.0, derr.z = 0.0; // 误差的微分
double x_scale_err, y_scale_err, z_scale_err;		 // 误差的量化因子
double x_scale_derr, y_scale_derr, z_scale_derr; // 误差微分的量化因子
double x_scale_kp, x_scale_ki, x_scale_kd;			 // delta (kp ki kd)的量化因子
double y_scale_kp, y_scale_ki, y_scale_kd;
double z_scale_kp, z_scale_ki, z_scale_kd;
double x_fpid_output, y_fpid_output, z_fpid_output;		 // 模糊PID的输出(增量)
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

// 输出速度调整系数
double k_vel_x = 6.0;
double k_vel_y = 6.3;
double k_vel_z = 3.6;

/*-------------------------------------
data_flag = 0  不记录数据
data_flag = 1  记录数据
---------------------------------------*/
int data_flag = 1;
ofstream oFile;

/*-------------------------------------
control_method = 0   程序原始的PI控制
control_method = 1   PID
control_method = 2   增量 PID
control_method = 3   模糊 PID
---------------------------------------*/
int control_method = 1;

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
	last_body = body;
	body.x = -detect.pose.position.y;
	body.y = detect.pose.position.x;
	body.z = -detect.pose.position.z;

	// 识别丢失情况
	if (body.y == -1.0)
	{
		body = last_body;
	}

	id = detect.id;
	num_all++;
}

// 数据记录
void data_record(int data_flag)
{
	if (data_flag == 1)
	{
		oFile.open("/home/itr/pid_data/pid_data.csv", ios::out | ios::app); // 数据续写非覆盖
		// 打印表头
		oFile << "err.x"
					<< ","
					<< "err.y"
					<< ","
					<< "body.z"
					<< ","
					<< "vel_x"
					<< ","
					<< "vel_y"
					<< ","
					<< "vel_z"
					<< ","
					<< "err_sum.x"
					<< ","
					<< "err_sum.y"
					<< ","
					<< "derr.x"
					<< ","
					<< "derr.y"
					<< ","
					<< "time" << endl; // endl 换行
	}
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
	data_record(data_flag);

	while (ros::ok())
	{
		switch (control_method)
		{
		// PI
		case 0:

			// 位置识别限幅
			range.Limit(body.x, 1, 0.5);
			range.Limit(body.y, 1, 0.5);

			// 位置误差
			err.x = 0.0 - body.x;
			err.y = 0.0 - body.y;
			err.z = H - body.z;

			// 位置误差积分
			err_sum.x += err.x;
			err_sum.y += err.y;
			err_sum.z += err.z;

			// 位置误差微分限幅
			range.Limit(err_sum.x, 5, 5);
			range.Limit(err_sum.y, 2, 2);
			range.Limit(err_sum.z, 5, 5);
			

			// 满足marker在摄像头内条件，进入降落第一阶段
			if (id != -1)
				flag = 1;

			// 顺利降落到相机坐标系下（0.3）左右，进入降落第二阶段
			if (flag == 1 && (-0.3) < err.x && err.x < 0.3 && (-0.3) < err.y && err.y < 0.3 && (-0.25) > body.z && body.z > (-1.1) && (id != -1))
			{
				flag = 2;
				H = 0;
				err_sum.z = 0;
			}

			// 顺利降落到相机坐标系下（0.1）左右，进入降落第三阶段
			if (flag == 2 && (-0.1) < err.x && err.x < 0.1 && (-0.1) < err.y && err.y < 0.1 && body.z > (-0.2) && (id != -1))
			{
				flag = 3;
			}

			if (id != -1)
			{
				detector_uav_vel_cmd.vel.linear.x = P_x * err.x + I_x * err_sum.x;
				detector_uav_vel_cmd.vel.linear.y = P_y * err.y + I_y * err_sum.y;
				if (body.z < (-0.3))
				{
					detector_uav_vel_cmd.vel.linear.z = P_z * err.z;
				}
				else
				{
					detector_uav_vel_cmd.vel.linear.z = 0.5 * err.z;
				}
				detector_uav_vel_cmd.flag = flag;
				detector_vel_pub.publish(detector_uav_vel_cmd);
			}
			else
			{
				detector_uav_vel_cmd.vel.linear.x = 0;
				detector_uav_vel_cmd.vel.linear.y = 0;
				detector_uav_vel_cmd.vel.linear.z = 0;
				detector_uav_vel_cmd.flag = 0;
				detector_vel_pub.publish(detector_uav_vel_cmd);
			}
			break;

		//-----------------------------------------------------------------------------------------
		// PID
		case 1:

			if (id != -1 && land == 0)
			{
				flag = 1;
			}

			// 时间微分
			c_time = ros::Time::now();
			current_time = c_time.toSec();
			dt = current_time - last_time;

			if (dt > 0.001)
			{
				// 误差e
				err.x = 0.0 - body.x;
				err.y = 0.0 - body.y;
				err.z = H - body.z;

				err_sum.x += err.x;
				err_sum.y += err.y;
				err_sum.z += err.z;

				range.Limit(err_sum.x, 2, 2);
				range.Limit(err_sum.y, 2, 2);
				range.Limit(err_sum.z, 0.5, 0.5);

				// 误差的微分ec
				derr.x = (err.x - last_err.x) / dt;
				derr.y = (err.y - last_err.y) / dt;
				derr.z = (err.z - last_err.z) / dt;

				detector_uav_vel_cmd.vel.linear.x = x_kp * err.x + x_ki * err_sum.x + x_kd * derr.x;
				detector_uav_vel_cmd.vel.linear.y = y_kp * err.y + y_ki * err_sum.y + y_kd * derr.y;
				detector_uav_vel_cmd.vel.linear.z = z_kp * err.z + z_ki * err_sum.z + z_kd * derr.z;
				detector_uav_vel_cmd.vel.linear.x = k_vel_x * detector_uav_vel_cmd.vel.linear.x;
				detector_uav_vel_cmd.vel.linear.y = k_vel_y * detector_uav_vel_cmd.vel.linear.y;
				detector_uav_vel_cmd.vel.linear.z = k_vel_z * detector_uav_vel_cmd.vel.linear.z;

				range.Limit(detector_uav_vel_cmd.vel.linear.x, 0.5, 0.5);
				range.Limit(detector_uav_vel_cmd.vel.linear.y, 0.5, 0.5);
				range.Limit(detector_uav_vel_cmd.vel.linear.z, 0.5, 0.5);

				// 顺利降落到相机坐标系下（0.3，0.3）左右，进入降落第一阶段
				if (flag == 1 && (-0.1) < err.x && err.x < 0.1 && (-0.1) < err.y && err.y < 0.1 && (-0.4) > body.z && body.z > (-1.0) && (id != -1) && status == 0)
				{
					land_num1++;
					if (land_num1 >= 250)
					{
						flag = 1;
						H = -0.2;
						status = 1;

						ROS_INFO("status1");
					}
				}

				if (status == 1 && body.z > (-0.3))
				{
					land_num2++;
					if (land_num2 >= 150)
					{
						ROS_INFO("status2");
						flag = 3;
					}
				}
				if (flag == 0)
				{
					detector_uav_vel_cmd.vel.linear.x = 0;
					detector_uav_vel_cmd.vel.linear.y = 0;
					detector_uav_vel_cmd.vel.linear.z = 0;
				}
				detector_uav_vel_cmd.flag = flag;
				detector_vel_pub.publish(detector_uav_vel_cmd);
			}
			last_err = err;
			last_time = current_time;
			break;

		//-----------------------------------------------------------------------------------------
		// 增量式PID
		case 2:

			if (id != -1)
			{
				flag = 1;
			}

			// 时间微分
			c_time = ros::Time::now();
			current_time = c_time.toSec();
			dt = current_time - last_time;

			if (dt > 0.001)
			{
				x_icm_pid.init(0.0, x_kp, x_ki, x_kd);
				y_icm_pid.init(0.0, y_kp, y_ki, y_kd);
				// z_icm_pid.init(H, z_kp, z_ki, z_kd);

				x_icm_vel = x_icm_pid.Incremental_PID_output(body.x);
				y_icm_vel = y_icm_pid.Incremental_PID_output(body.y);
				// z_icm_vel = z_icm_pid.Incremental_PID_output(body.z);

				detector_uav_vel_cmd.vel.linear.x += 4.6 * x_icm_vel;
				detector_uav_vel_cmd.vel.linear.y += 4.6 * y_icm_vel;

				err.z = H - body.z;
				err_sum.z += err.z;
				derr.z = (err.z - last_err.z) / dt;

				range.Limit(err_sum.z, 1, 0.5);

				detector_uav_vel_cmd.vel.linear.z = z_kp * err.z + z_ki * err_sum.z + z_kd * derr.z;
				detector_uav_vel_cmd.vel.linear.z = k_vel_z * detector_uav_vel_cmd.vel.linear.z;

				range.Limit(detector_uav_vel_cmd.vel.linear.x, 0.5, 0.5);
				range.Limit(detector_uav_vel_cmd.vel.linear.y, 0.5, 0.5);

				// 顺利降落到相机坐标系下（0.3，0.3）左右，进入降落第一阶段
				if (flag == 1 && (-0.1) < err.x && err.x < 0.1 && (-0.1) < err.y && err.y < 0.1 && (-0.4) > body.z && body.z > (-1.0) && (id != -1) && status == 0)
				{
					land_num1++;
					if (land_num1 >= 200)
					{
						flag = 1;
						H = -0.2;
						status = 1;

						ROS_INFO("status1");
					}
				}

				if (status == 1)
				{
					land_num2++;
					if (land_num2 >= 85)
					{
						ROS_INFO("status2");
						flag = 3;
					}
				}
				if (flag == 0)
				{
					detector_uav_vel_cmd.vel.linear.x = 0;
					detector_uav_vel_cmd.vel.linear.y = 0;
					detector_uav_vel_cmd.vel.linear.z = 0;
				}
				ROS_INFO("H = %f", H);
				detector_uav_vel_cmd.flag = flag;
				detector_vel_pub.publish(detector_uav_vel_cmd);
			}
			last_time = current_time;
			break;

		//-----------------------------------------------------------------------------------------
		//  模糊PID
		case 3:

			if (id != -1)
			{
				flag = 1;
			}

			c_time = ros::Time::now();
			current_time = c_time.toSec();
			dt = current_time - last_time;

			if (dt > 0.001)
			{

				// 误差e(输入1)
				err.x = 0.0 - body.x;
				err.y = 0.0 - body.y;

				// 误差的微分ec(输入2)
				derr.x = (err.x - last_err.x) / dt;
				derr.y = (err.y - last_err.y) / dt;

				// 赋PID初值
				if (fuzzy_flag == 0)
				{

					ROS_INFO("fuzzy_flag changed!");
					x_fuzzy_pid.init(0.0, x_kp, x_ki, x_kd);
					y_fuzzy_pid.init(0.0, y_kp, y_ki, y_kd);
					fuzzy_flag = 1;
				}

				// delta(P I D)和两个输入的量化因子(需要调整)
				x_scale_kp = 0.036 / 3, x_scale_ki = 0.0002 / 3, x_scale_kd = 0.005 / 3, x_scale_err = 15, x_scale_derr = 15 * dt;
				y_scale_kp = 0.036 / 3, y_scale_ki = 0.0002 / 3, y_scale_kd = 0.005 / 3, y_scale_err = 15, y_scale_derr = 15 * dt;

				x_fzy_fpid = x_fuzzy_pid.Fuzzy_PID_output(body.x, err.x, derr.x, x_scale_kp, x_scale_ki, x_scale_kd, x_scale_err, x_scale_derr, x_fuzzy_icm_pid);
				y_fzy_fpid = y_fuzzy_pid.Fuzzy_PID_output(body.y, err.y, derr.y, y_scale_kp, y_scale_ki, y_scale_kd, y_scale_err, y_scale_derr, y_fuzzy_icm_pid);

				x_fzy_pid.init(0.0, x_fzy_fpid[0], x_fzy_fpid[1], x_fzy_fpid[2]);
				y_fzy_pid.init(0.0, y_fzy_fpid[0], y_fzy_fpid[1], y_fzy_fpid[2]);

				x_fpid_output = x_fzy_pid.Incremental_PID_output(body.x);
				y_fpid_output = y_fzy_pid.Incremental_PID_output(body.y);

				//输出的量化因子(需要调整)
				x_scale_output = 3.0;
				y_scale_output = 3.0;

				detector_uav_vel_cmd.vel.linear.x += x_fpid_output * x_scale_output;
				detector_uav_vel_cmd.vel.linear.y += y_fpid_output * y_scale_output;

				err.z = H - body.z;
				err_sum.z += err.z;
				derr.z = (err.z - last_err.z) / dt;
				range.Limit(err_sum.z, 1, 0.5);

				detector_uav_vel_cmd.vel.linear.z = z_kp * err.z + z_ki * err_sum.z + z_kd * derr.z;
				detector_uav_vel_cmd.vel.linear.z = k_vel_z * detector_uav_vel_cmd.vel.linear.z;

				// 顺利降落到相机坐标系下（0.3，0.3）左右，进入降落第一阶段
				if (flag == 1 && (-0.1) < err.x && err.x < 0.1 && (-0.1) < err.y && err.y < 0.1 && (-0.4) > body.z && body.z > (-1.0) && (id != -1) && status == 0)
				{
					land_num1++;
					if (land_num1 >= 250)
					{
						flag = 1;
						H = -0.25;
						status = 1;

						ROS_INFO("status1");
					}
				}

				if (status == 1)
				{
					land_num2++;
					if (land_num2 >= 25)
					{
						ROS_INFO("status2");
						flag = 3;
					}
				}

				if (flag == 0)
				{
					detector_uav_vel_cmd.vel.linear.x = 0;
					detector_uav_vel_cmd.vel.linear.y = 0;
					detector_uav_vel_cmd.vel.linear.z = 0;
				}

				range.Limit(detector_uav_vel_cmd.vel.linear.x, 0.5, 0.5);
				range.Limit(detector_uav_vel_cmd.vel.linear.y, 0.5, 0.5);

				detector_uav_vel_cmd.flag = flag;
				detector_vel_pub.publish(detector_uav_vel_cmd);
			}

			last_time = current_time;
			last_err.x = err.x;
			last_err.y = err.y;
			last_err.z = err.z;

			break;

		default:
			break;
		}
		ROS_INFO("*****************************************************");
		ROS_INFO("control_method:%d", control_method);
		ROS_INFO("flag:%d", detector_uav_vel_cmd.flag);
		ROS_INFO("target_H = %f", H);
		ROS_INFO("err.x:%f", err.x);
		ROS_INFO("err.y:%f", err.y);
		ROS_INFO("body.z:%f", body.z);
		ROS_INFO("vel_x:%f", detector_uav_vel_cmd.vel.linear.x);
		ROS_INFO("vel_y:%f", detector_uav_vel_cmd.vel.linear.y);
		ROS_INFO("vel_z:%f", detector_uav_vel_cmd.vel.linear.z);
		ROS_INFO("*****************************************************");

		if (data_flag == 1)
		{
			oFile << err.x 
						<< "," 
						<< err.y 
						<< "," 
						<< body.z 
						<< "," 
						<< detector_uav_vel_cmd.vel.linear.x 
						<< "," 
						<< detector_uav_vel_cmd.vel.linear.y 
						<< "," 
						<< detector_uav_vel_cmd.vel.linear.z 
						<< "," 
						<< err_sum.x 
						<< "," 
						<< err_sum.y 
						<< "," 
						<< derr.x 
						<< "," 
						<< derr.y 
						<< "," 
						<< current_time 
						<< endl; // endl 换行
		}
		ros::spinOnce();
		rate.sleep();
	}
	oFile.close();
	return 0;
}
