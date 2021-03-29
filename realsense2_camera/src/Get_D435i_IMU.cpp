#include <Get_D435i_IMU/Get_D435i_IMU.h>

Get_D435i_IMU::Get_D435i_IMU(ros::NodeHandle &nh)
{
    this->nh = &nh;
    IMUgyroData_Subscriber = nh.subscribe("/camera/gyro/sample", 1, &Get_D435i_IMU::GetIMUgyroDataFunction,this);
    IMUgyroData_Publisher = nh.advertise<realsense2_camera::IMUdata>("/camera/gyro/IMUdata", 10);
    gyro_data.x =0.0;
    gyro_data.y =0.0;
    gyro_data.z =0.0;
};

Get_D435i_IMU::~Get_D435i_IMU(){

};

void Get_D435i_IMU::GetIMUgyroDataFunction(const sensor_msgs::Imu::ConstPtr& msg)
{
    gyro_data.x = msg->angular_velocity.x;
    gyro_data.y = msg->angular_velocity.y;
    gyro_data.z = msg->angular_velocity.z;
    msgtime = msg->header.stamp;
    get_imu_gyro_data = true;
    // std::cout << "pitch:" << gyro_data.x << "yaw: " << gyro_data.y << "roll: " << gyro_data.z<< std::endl;
}

// Returns the current rotation angle
float3 Get_D435i_IMU::get_theta()
{
    std::lock_guard<std::mutex> lock(theta_mtx);
    return theta;
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

    // Compute the difference between arrival times of previous and current gyro frames
    double dt_gyro = (ts - last_ts_gyro).toSec();

    if(dt_gyro>0.02)
    {
        dt_gyro = 0.015;
    }
    // ROS_INFO("dt_gyro = %f",dt_gyro);
    last_ts_gyro = ts;
    // Change in angle equals gyro measures * time passed since last measurement
    gyro_angle = gyro_angle * dt_gyro;
    // Apply the calculated change of angle to the current angle (theta)
    std::lock_guard<std::mutex> lock(theta_mtx);
    theta.add(-gyro_angle.z, -gyro_angle.y, gyro_angle.x);

}

float Get_D435i_IMU::normalize_angle(float phi) 
{
  //Normalize phi to be between -pi and pi
    while(phi > 180) {
        phi = phi - 2 * 180;
    }

    while(phi < -180) {
        phi = phi + 2 * 180;
    }
    return phi;
}


void Get_D435i_IMU::GetImudata() 
{
    process_gyro(gyro_data,msgtime);
    float3 th = get_theta();

    // ROS_INFO("roll: %f, pitch: %f, yaw: %f",normalize_angle(th.z*(180.0/PI)),
    //                                         normalize_angle(th.x*(180.0/PI)),
    //                                         normalize_angle(th.y*(180.0/PI)));
      
    msg_imudata.roll = normalize_angle(th.z*(180.0/PI));
    msg_imudata.pitch = normalize_angle(th.x*(180.0/PI));
    msg_imudata.yaw = normalize_angle(th.y*(180.0/PI));
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


