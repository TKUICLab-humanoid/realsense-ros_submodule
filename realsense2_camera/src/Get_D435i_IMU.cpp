#include <Get_D435i_IMU/Get_D435i_IMU.h>

Get_D435i_IMU::Get_D435i_IMU(ros::NodeHandle &nh)
{
    this->nh = &nh;
    IMUgyroData_Subscriber = nh.subscribe("/camera/imu", 1, &Get_D435i_IMU::GetIMUgyroDataFunction,this);
    IMUgyroData_Publisher = nh.advertise<realsense2_camera::IMUdata>("/camera/gyro/IMUdata", 10);
    gyro_data.x =0.0;
    gyro_data.y =0.0;
    gyro_data.z =0.0;
};

Get_D435i_IMU::~Get_D435i_IMU(){

};

void Get_D435i_IMU::GetIMUgyroDataFunction(const sensor_msgs::Imu::ConstPtr& msg)
{
    // std::cout << "pitch:" << msg->angular_velocity.x << "yaw: " << msg->angular_velocity.y << "roll: " << msg->angular_velocity.z<< std::endl;
    gyro_data.x = msg->angular_velocity.x;
    gyro_data.y = msg->angular_velocity.y;
    gyro_data.z = msg->angular_velocity.z;
    accel_data.x = msg->linear_acceleration.x;
    accel_data.y = msg->linear_acceleration.y;
    accel_data.z = msg->linear_acceleration.z;
    msgtime = msg->header.stamp;
    get_imu_gyro_data = true;
    // std::cout << "pitch:" << gyro_data.x << "yaw: " << gyro_data.y << "roll: " << gyro_data.z<< std::endl;
}

// Returns the current rotation angle
float3 Get_D435i_IMU::get_theta_gyro()
{
    std::lock_guard<std::mutex> lock(theta_mtx);
    return theta_gyro;
}
float3 Get_D435i_IMU::get_theta_accel()
{
    std::lock_guard<std::mutex> lock(theta_mtx);
    return theta_accel;
}
void Get_D435i_IMU::process_gyro(rs2_vector Gyro_data,ros::Time ts)
{
    if (firstGyro) // On the first iteration, use only data from accelerometer to set the camera's initial position
    {
        firstGyro = false;

        last_ts_gyro = ts;
        return;
    }
    // Holds the change in angle, as calculated from gyro
    float3 gyro_angle;
    // Initialize gyro_angle with data from gyro
    gyro_angle.x = Gyro_data.x; // Pitch
    gyro_angle.y = Gyro_data.y; // Yaw
    gyro_angle.z = Gyro_data.z; // Roll
    std::cout << "pitch:" << gyro_angle.x << "yaw: " << gyro_angle.y << "roll: " << gyro_angle.z<< std::endl;
    // if(abs(gyro_angle.x)<0.006 && abs(gyro_angle.x)>0.0)
    // {
    //     gyro_angle.x = 0.00000;
    // }
    if(abs(gyro_angle.y)<0.005 && abs(gyro_angle.y)>0.0)
    {
        gyro_angle.y = 0.00001;
    }
    if(abs(gyro_angle.z)<0.00695 && abs(gyro_angle.z)>0.0)
    {
        gyro_angle.z = 0.0000001;
    }
    // Compute the difference between arrival times of previous and current gyro frames
    double dt_gyro = (ts - last_ts_gyro).toSec();

    if(dt_gyro>0.03)
    {
        dt_gyro = 0.015;
    }
    ROS_INFO("dt_gyro = %f",dt_gyro);
    last_ts_gyro = ts;
    // Change in angle equals gyro measures * time passed since last measurement
    gyro_angle = gyro_angle * dt_gyro;
   
    // Apply the calculated change of angle to the current angle (theta)
    std::lock_guard<std::mutex> lock(theta_mtx);
    theta_gyro.add(-gyro_angle.z, -gyro_angle.y, gyro_angle.x);

}

void Get_D435i_IMU::process_accel(rs2_vector Accel_data)
{
    // Holds the angle as calculated from accelerometer data
    float3 accel_angle;

    // Calculate rotation angle from accelerometer data
    accel_angle.z = atan2(Accel_data.y, Accel_data.z);
    accel_angle.x = atan2(Accel_data.x, sqrt(Accel_data.y * Accel_data.y + Accel_data.z * Accel_data.z));

    // If it is the first iteration, set initial pose of camera according to accelerometer data (note the different handling for Y axis)
    std::lock_guard<std::mutex> lock(theta_mtx);
    if (firstAccel)
    {
        firstAccel = false;
        theta_accel = accel_angle;
        // Since we can't infer the angle around Y axis using accelerometer data, we'll use PI as a convetion for the initial pose
        theta_accel.y = PI;
    }
    else
    {
        /* 
        Apply Complementary Filter:
            - high-pass filter = theta * alpha:  allows short-duration signals to pass through while filtering out signals
                that are steady over time, is used to cancel out drift.
            - low-pass filter = accel * (1- alpha): lets through long term changes, filtering out short term fluctuations 
        */
        theta_accel.x = theta_accel.x * alpha + accel_angle.x * (1 - alpha);
        theta_accel.z = theta_accel.z * alpha + accel_angle.z * (1 - alpha);
    }
}

float Get_D435i_IMU::normalize_angle(float phi) 
{
  //Normalize phi to be between -pi and pi
    while(phi > 180.0) {
        phi = phi - 2 * 180.0;
    }

    while(phi < -180) {
        phi = phi + 2 * 180.0;
    }
    return phi;
}


void Get_D435i_IMU::GetImudata() 
{
    process_gyro(gyro_data,msgtime);
    process_accel(accel_data);
    float3 tth = get_theta_accel();
    float3 th = get_theta_gyro();



    // ROS_INFO("gyro roll: %f, pitch: %f, yaw: %f",normalize_angle(th.z*RAD2DEG),
    //                                              normalize_angle(th.x*RAD2DEG),
    //                                              normalize_angle(th.y*RAD2DEG));
    

    // ROS_INFO("accel roll: %f, pitch: %f, yaw: %f",normalize_angle(tth.z*RAD2DEG),
    //                                               normalize_angle(tth.x*RAD2DEG),
    //                                               normalize_angle(tth.y*RAD2DEG));
      
    if(round(normalize_angle(tth.z * RAD2DEG)) == -90.0)
    {
        theta_gyro.x = 0.0;
    }
    // if( normalize_angle(tth.z*RAD2DEG) == -90.0)
    // {
    //     last_z = th.z;
    // }
    // if( abs(normalize_angle(tth.x*RAD2DEG)) <= 0.3 && abs(normalize_angle(tth.x*RAD2DEG)) >=0.0)
    // {
    //     th.x = tth.x;
    //     theta_gyro.x =  tth.x;
    // }   
    // ROS_INFO("gyro roll: %f, pitch: %f, yaw: %f",normalize_angle(tth.z*RAD2DEG),
    //                                              normalize_angle(tth.x*RAD2DEG),
    //                                              normalize_angle(th.y*RAD2DEG));
    msg_imudata.roll = normalize_angle(th.z * RAD2DEG);
    msg_imudata.pitch = normalize_angle(tth.x * RAD2DEG);
    msg_imudata.yaw = normalize_angle(th.y * RAD2DEG);
    IMUgyroData_Publisher.publish(msg_imudata);
}


int main(int argc, char **argv)try
{
    ros::init(argc, argv, "realsense_IMU");
    ros::NodeHandle nh;
    Get_D435i_IMU get_D435i_IMU(nh);
    ros::Rate loop_rate(60);

    while (nh.ok())
    {
        get_D435i_IMU.GetImudata();                                 
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}


