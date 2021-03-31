#pragma once
#include <librealsense2/rs.hpp>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <iomanip>
#include <cmath>
#include <map>
#include <functional>
#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <realsense2_camera/IMUdata.h>
#include <sys/time.h>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"

using namespace std;

using namespace cv;
#define width 640 
#define height 480 
#define fps 30
#ifndef PI
const double PI = 3.14159265358979323846;
#endif
#define RAD2DEG         180/PI
struct float3 {
    float x, y, z;
    float3 operator*(float t)
    {
        return { x * t, y * t, z * t };
    }

    float3 operator-(float t)
    {
        return { x - t, y - t, z - t };
    }

    void operator*=(float t)
    {
        x = x * t;
        y = y * t;
        z = z * t;
    }

    void operator=(float3 other)
    {
        x = other.x;
        y = other.y;
        z = other.z;
    }

    void add(float t1, float t2, float t3)
    {
        x += t1;
        y += t2;
        z += t3;
    }
};
class Get_D435i_IMU
{
private:
    ros::NodeHandle *nh;
    ros::Subscriber IMUgyroData_Subscriber;
    realsense2_camera::IMUdata msg_imudata;
    // theta is the angle of camera rotation in x, y and z components
    bool get_imu_gyro_data;
    float3 theta_gyro = {0.0,0.0,0.0};
    float3 theta_accel = {0.0,0.0,0.0};
    std::mutex theta_mtx;
    /* alpha indicates the part that gyro and accelerometer take in computation of theta; higher alpha gives more weight to gyro, but too high
    values cause drift; lower alpha gives more weight to accelerometer, which is more sensitive to disturbances */
    float alpha = 0.98;
    bool firstGyro = true;
    bool firstAccel = true;
    ros::Time msgtime;
    // Keeps the arrival time of previous gyro frame
    ros::Time last_ts_gyro;
    ros::Publisher IMUgyroData_Publisher;
    float last_z;
public:
    rs2_vector gyro_data;
    rs2_vector accel_data;
    Get_D435i_IMU(ros::NodeHandle &nh);
    ~Get_D435i_IMU();
    void process_gyro(rs2_vector gyro_data,ros::Time ts);
    void process_accel(rs2_vector accel_data);
    float3 get_theta_gyro();
    float3 get_theta_accel();
    void GetIMUgyroDataFunction(const sensor_msgs::Imu::ConstPtr& msg);
    float normalize_angle(float phi);
    void GetImudata();
    
};
extern Get_D435i_IMU* get_D435i_IMU;