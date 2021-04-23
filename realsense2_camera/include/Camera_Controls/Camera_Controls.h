#include <ros/ros.h>
#include <iostream>
#include <iomanip>
#include <map>
#include <utility>
#include <vector>
#include <librealsense2/rs.hpp>

#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include "tku_msgs/camera.h"
#include "tku_msgs/CameraInfo.h"
#include "RGB_camera_controls/RGB_camera_controls.h"
#include "base_realsense_node.h"
#include "realsense_node_factory.h"

using namespace realsense2_camera;

class Camera_control
{
    private:
        ros::Subscriber getpara_sub_;
        ros::ServiceServer Camera_service;
        // rs2::context ctx; 
        // rs2::device_list devices;
        ros::NodeHandle *nh;
    public:
        rs2::sensor sensor; 
        Camera_control( ros::NodeHandle& nh);
        ~Camera_control();
        float exposure_, brightness_, contrast_, saturation_,white_balance_;
        bool auto_exposure_, auto_white_balance_,auto_Backlight_Compensation_;
        void loadCameraFile();
        void saveCameraFile();
        // void print_device_information();
        inline void print_separator();
        rs2::sensor get_sensor_from_a_device(); 
        // void get_sensor_all_option_value();
        bool CallCameraInfoFunction(tku_msgs::CameraInfo::Request &req, tku_msgs::CameraInfo::Response &res);
        void SetCameraParameter();
        void getparaCallback(const tku_msgs::camera &msg);

};
extern Camera_control* camera_control;