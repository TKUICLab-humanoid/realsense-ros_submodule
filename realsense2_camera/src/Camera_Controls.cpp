#include "Camera_Controls/Camera_Controls.h"

//Camera_control* camera_control = new Camera_control(ros::NodeHandle &nh);

Camera_control::Camera_control(ros::NodeHandle& nh):_node_handle(nh)
{
    image_transport::ImageTransport it(_node_handle);    
    getpara_sub_ = _node_handle.subscribe("/Camera_Topic", 1, &Camera_control::getparaCallback,this);
    savepara_sub_ = _node_handle.subscribe("/realsense/savecamera", 1, &Camera_control::setparaCallback,this);

    Camera_service = _node_handle.advertiseService("/CameraInfo", &Camera_control::CallCameraInfoFunction,this);
    _cameraparam_publisher = _node_handle.advertise<tku_msgs::Cameraparam>("/realsense/setcamera", 10);
    
    // exposure_ = 0;
    // brightness_ = 0;
    // contrast_ = 50;
    // saturation_ = 50;
    // white_balance_ = 4000;
    // auto_exposure_ = 0;
    // auto_white_balance_ = 0;
    // auto_Backlight_Compensation_ = 0;
    // _exposure_ = 0;
    // _brightness_ = 0;
    // _contrast_ = 50;
    // _saturation_ = 50;
    // _white_balance_ = 4000;
    // _auto_exposure_ = 0;
    // _auto_white_balance_ = 0;
    // _auto_Backlight_Compensation_ = 0;
}
Camera_control::~Camera_control()
{
    
}

void Camera_control::loadCameraFile()
{
    RGB_camera_controls->LoadCameraSetFile();
    auto_Backlight_Compensation_ = (RGB_camera_controls->CameraParameterValue->auto_Backlight_Compensation==1)?true:false;
    auto_exposure_      = (RGB_camera_controls->CameraParameterValue->auto_exposure==1)?true:false;
    auto_white_balance_ = (RGB_camera_controls->CameraParameterValue->auto_white_balance==1)?true:false;
    brightness_         = RGB_camera_controls->CameraParameterValue->brightness;
    contrast_           = RGB_camera_controls->CameraParameterValue->contrast;
    saturation_         = RGB_camera_controls->CameraParameterValue->saturation;
    white_balance_      = RGB_camera_controls->CameraParameterValue->white_balance;
} 

void Camera_control::getparaCallback(const tku_msgs::camera &msg)
{
    brightness_         = int(msg.brightness);
    contrast_           = int(msg.contrast);
    saturation_         = int(msg.saturation);
    white_balance_      = int(msg.white_balance);
    auto_white_balance_ = msg.auto_white_balance;
    auto_exposure_       = msg.auto_exposure; 
    auto_Backlight_Compensation_ = msg.auto_Backlight_Compensation;
    
    tku_msgs::Cameraparam setparam_msg;
    setparam_msg.brightness = brightness_;
    setparam_msg.contrast = contrast_;
    setparam_msg.saturation = saturation_;
    setparam_msg.white_balance = white_balance_;
    setparam_msg.auto_exposure =  auto_exposure_;
    setparam_msg.auto_white_balance = auto_white_balance_;
    setparam_msg.auto_Backlight_Compensation = auto_Backlight_Compensation_;
    _cameraparam_publisher.publish(setparam_msg);
    // SetCameraParameter();  
}



void Camera_control::setparaCallback(const tku_msgs::Cameraparam &msg)
{
    _brightness_         = int(msg.brightness);
    _contrast_           = int(msg.contrast);
    _saturation_         = int(msg.saturation);
    _white_balance_      = int(msg.white_balance);
    _auto_white_balance_ = msg.auto_white_balance;
    _auto_exposure_       = msg.auto_exposure; 
    _auto_Backlight_Compensation_ = msg.auto_Backlight_Compensation;
    saveCameraFile();
}

void Camera_control::saveCameraFile()
{
    // ROS_INFO("brightness                        = %d",_brightness_);
    // ROS_INFO("contrast                          = %d",_contrast_);
    // ROS_INFO("saturation                        = %d",_saturation_);
    // ROS_INFO("white_balance                     = %d",_white_balance_);
    // ROS_INFO("auto_white_balance                = %d",(_auto_white_balance_==true)?1:0);
    // ROS_INFO("auto_exposure                     = %d",(_auto_exposure_==true)?1:0);
    // ROS_INFO("auto_Backlight_Compensation       = %d",(_auto_Backlight_Compensation_==true)?1:0);
    // ROS_INFO("===================================");
    
    RGB_camera_controls->CameraParameterValue->brightness         = _brightness_;
    RGB_camera_controls->CameraParameterValue->contrast           = _contrast_;
    RGB_camera_controls->CameraParameterValue->saturation         = _saturation_;
    RGB_camera_controls->CameraParameterValue->white_balance      = _white_balance_;
    RGB_camera_controls->CameraParameterValue->auto_exposure       = (_auto_exposure_==1)?true:false;
    RGB_camera_controls->CameraParameterValue->auto_white_balance = (_auto_white_balance_==1)?true:false;
    RGB_camera_controls->CameraParameterValue->auto_Backlight_Compensation = (_auto_Backlight_Compensation_==1)?true:false;
    RGB_camera_controls->SaveCameraSetFile();
}

bool Camera_control::CallCameraInfoFunction(tku_msgs::CameraInfo::Request &req, tku_msgs::CameraInfo::Response &res)
{
    res.brightness         = brightness_;
    res.contrast           = contrast_;
    res.saturation         = saturation_;
    res.white_balance      = white_balance_;
    res.auto_exposure       = auto_exposure_;
    res.auto_white_balance = auto_white_balance_;
    res.auto_Backlight_Compensation = auto_Backlight_Compensation_;
    return true;
}

// void Camera_control::SetCameraParameter()
// {
//     rs2::context ctx; 
//     rs2::device dev;
//     auto list = ctx.query_devices();
//     if (list.size() == 0)
//     {
//         ROS_WARN("No RealSense devices were found!");
//     }else{
//         try
//         {
//             dev = list[0];
//         }
//         catch(const std::exception& ex)
//         {
//             ROS_WARN_STREAM("Device /" << list.size() << " failed with exception: " << ex.what());
//         }
//         std::vector<rs2::sensor> sensors = dev.query_sensors();
//         sensor = sensors[1];
//     }
    

//     // for(rs2::sensor sensor : _dev_sensors_copy)
//     // {
//         rs2_option option_type;
//         if (brightness_ >= -64 )
//         {
//             option_type = static_cast<rs2_option>(1);
//             try
//             {
//                 sensor.set_option(option_type, brightness_);
//             }
//             catch (const rs2::error& e)
//             {
//                 std::cerr << "Failed to set option " << option_type << ". (" << e.what() << ")" << std::endl;
//             }
//         }
//         if (contrast_ >= 0)
//         {
//             option_type = static_cast<rs2_option>(2);
//             try
//             {
//                 sensor.set_option(option_type, contrast_);
//             }
//             catch (const rs2::error& e)
//             {
//                 std::cerr << "Failed to set option " << option_type << ". (" << e.what() << ")" << std::endl;
//             }
//         }
//         if (saturation_ >= 0 )
//         {
//             option_type = static_cast<rs2_option>(7);
//             try
//             {
//                 sensor.set_option(option_type, saturation_);
//             }
//             catch (const rs2::error& e)
//             {
//                 std::cerr << "Failed to set option " << option_type << ". (" << e.what() << ")" << std::endl;
//             }
//         }
//         if(auto_white_balance_ == 0 )
//         {
//             option_type = static_cast<rs2_option>(11);
//             sensor.set_option(option_type, 0);
//             if (white_balance_ >= 2800 )
//             {
//                 option_type = static_cast<rs2_option>(9);
//                 try
//                 {
//                     sensor.set_option(option_type, white_balance_);
//                 }
//                 catch (const rs2::error& e)
//                 {
//                     std::cerr << "Failed to set option " << option_type << ". (" << e.what() << ")" << std::endl;
//                 }  
//             }
//         }else
//         {
//             option_type = static_cast<rs2_option>(11);
//             try
//             {
//                 sensor.set_option(option_type, 1);
//             }
//             catch (const rs2::error& e)
//             {
//                 std::cerr << "Failed to set option " << option_type << ". (" << e.what() << ")" << std::endl;
//             }  
//         }
//         if (auto_exposure_ == 1 )
//         {
//             option_type = static_cast<rs2_option>(10);
//             sensor.set_option(option_type, 1);
//         }else
//         {
//             option_type = static_cast<rs2_option>(10);
//             sensor.set_option(option_type, 0);
//         }
//         if (auto_Backlight_Compensation_ == 1 )
//         {
//             option_type = static_cast<rs2_option>(0);
//             sensor.set_option(option_type, 1);
//         }else
//         {
//             option_type = static_cast<rs2_option>(0);
//             sensor.set_option(option_type, 0);
//         }
//         saveCameraFile();
//     // }
    
// }




int main(int argc, char **argv)
{
    ros::init(argc, argv, "realsense_cam");
    ros::NodeHandle nh;
    // rs2::device dev;
    // ros::NodeHandle privateNodeHandle;
    // RealSenseNodeFactory realSenseNodeFactory;
    // ros::NodeHandle nh = realSenseNodeFactory.getNodeHandle();
	// auto privateNh = realSenseNodeFactory.getPrivateNodeHandle();
    
    Camera_control camera_control(nh);
    camera_control.loadCameraFile();
    ros::Rate loop_rate(60);
    
    while (nh.ok())
    {
        //camera_control->print_separator();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}