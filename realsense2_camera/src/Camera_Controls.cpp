#include "Camera_Controls/Camera_Controls.h"

//Camera_control* camera_control = new Camera_control(ros::NodeHandle &nh);

Camera_control::Camera_control(ros::NodeHandle& nh)
{
    this->nh = &nh;
    image_transport::ImageTransport it(nh);    
    getpara_sub_ = nh.subscribe("/Camera_Topic", 1, &Camera_control::getparaCallback,this);
    Camera_service = nh.advertiseService("/CameraInfo", &Camera_control::CallCameraInfoFunction,this);
    
    exposure_ = 0;
    brightness_ = 0;
    contrast_ = 0;
    saturation_ = 0;
    white_balance_ = 0;
    auto_exposure_ = 0;
    auto_white_balance_ = 0;
    auto_Backlight_Compensation_ = 0;
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
void Camera_control::saveCameraFile()
{
    ROS_INFO("brightness                        = %f",brightness_);
    ROS_INFO("contrast                          = %f",contrast_);
    ROS_INFO("saturation                        = %f",saturation_);
    ROS_INFO("white_balance                     = %f",white_balance_);
    ROS_INFO("auto_white_balance                = %d",(auto_white_balance_==true)?1:0);
    ROS_INFO("auto_exposure                     = %d",(auto_exposure_==true)?1:0);
    ROS_INFO("auto_Backlight_Compensation       = %d",(auto_Backlight_Compensation_==true)?1:0);
    ROS_INFO("===================================");
    
    RGB_camera_controls->CameraParameterValue->brightness         = brightness_;
    RGB_camera_controls->CameraParameterValue->contrast           = contrast_;
    RGB_camera_controls->CameraParameterValue->saturation         = saturation_;
    RGB_camera_controls->CameraParameterValue->white_balance      = white_balance_;
    RGB_camera_controls->CameraParameterValue->auto_exposure       = (auto_exposure_==1)?true:false;
    RGB_camera_controls->CameraParameterValue->auto_white_balance = (auto_white_balance_==1)?true:false;
    RGB_camera_controls->CameraParameterValue->auto_Backlight_Compensation = (auto_Backlight_Compensation_==1)?true:false;
    RGB_camera_controls->SaveCameraSetFile();
}

// void Camera_control::print_device_information()
// {
//     rs2::context ctx; 
//     rs2::device_list devices = ctx.query_devices();
//     rs2::device selected_device = devices[0];
//     std::cout << "Device information: " << std::endl;
//     for (int i = 0; i < static_cast<int>(RS2_CAMERA_INFO_COUNT); i++)
//     {
//         rs2_camera_info info_type = static_cast<rs2_camera_info>(i);
//         std::cout << "  " << std::left << std::setw(20) << info_type << " : ";
//         if (selected_device.supports(info_type))
//             std::cout << selected_device.get_info(info_type) << std::endl;
//         else
//             std::cout << "N/A" << std::endl;
//     }
// }

inline void Camera_control::print_separator()
{
    std::cout << "\n======================================================\n" << std::endl;
}

rs2::sensor Camera_control::get_sensor_from_a_device()
{
    rs2::context ctx; 
    rs2::device dev;
    auto list = ctx.query_devices();
    if (list.size() == 0)
    {
        ROS_WARN("No RealSense devices were found!");
    }else{
        try
        {
            dev = list[0];
        }
        catch(const std::exception& ex)
        {
            ROS_WARN_STREAM("Device /" << list.size() << " failed with exception: " << ex.what());
        }
        std::vector<rs2::sensor> sensors = dev.query_sensors();
        //RGB camera
        return  sensors[1];
    } 
}

// void Camera_control::get_sensor_all_option_value()
// {
//     std::cout << "Sensor supports the following options:\n" << std::endl;
//     for (int i = 0; i < static_cast<int>(RS2_OPTION_COUNT); i++)
//     {
//         if(i<12)
//         {
//             rs2_option option_type = static_cast<rs2_option>(i);
//             std::cout << "  " << i << ": " << option_type;
//             if (sensor.supports(option_type))
//             {
//                 float current_value = sensor.get_option(option_type);
//                 rs2::option_range range = sensor.get_option_range(option_type);
//                 float default_value = range.def;
//                 float maximum_supported_value = range.max;
//                 float minimum_supported_value = range.min;
//                 float difference_to_next_value = range.step;
//                 std::cout << std::endl << "  Current Value : " << current_value << std::endl;
//                 std::cout << "  Min Value     : " << minimum_supported_value << std::endl;
//                 std::cout << "  Max Value     : " << maximum_supported_value << std::endl;
//                 print_separator();
//             }
//             else
//             {
//                 std::cout << " is not supported" << std::endl;
//             }
//         }else{
//             break;
//         }
//     }
// }

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

void Camera_control::SetCameraParameter()
{
    // for(rs2::sensor sensor : _dev_sensors_copy)
    // {
        rs2_option option_type;
        if (brightness_ >= -64 )
        {
            option_type = static_cast<rs2_option>(1);
            try
            {
                sensor.set_option(option_type, brightness_);
            }
            catch (const rs2::error& e)
            {
                std::cerr << "Failed to set option " << option_type << ". (" << e.what() << ")" << std::endl;
            }
        }
        if (contrast_ >= 0)
        {
            option_type = static_cast<rs2_option>(2);
            try
            {
                sensor.set_option(option_type, contrast_);
            }
            catch (const rs2::error& e)
            {
                std::cerr << "Failed to set option " << option_type << ". (" << e.what() << ")" << std::endl;
            }
        }
        if (saturation_ >= 0 )
        {
            option_type = static_cast<rs2_option>(7);
            try
            {
                sensor.set_option(option_type, saturation_);
            }
            catch (const rs2::error& e)
            {
                std::cerr << "Failed to set option " << option_type << ". (" << e.what() << ")" << std::endl;
            }
        }
        if(auto_white_balance_ == 0 )
        {
            option_type = static_cast<rs2_option>(11);
            sensor.set_option(option_type, 0);
            if (white_balance_ >= 2800 )
            {
                option_type = static_cast<rs2_option>(9);
                try
                {
                    sensor.set_option(option_type, white_balance_);
                }
                catch (const rs2::error& e)
                {
                    std::cerr << "Failed to set option " << option_type << ". (" << e.what() << ")" << std::endl;
                }  
            }
        }else
        {
            option_type = static_cast<rs2_option>(11);
            try
            {
                sensor.set_option(option_type, 1);
            }
            catch (const rs2::error& e)
            {
                std::cerr << "Failed to set option " << option_type << ". (" << e.what() << ")" << std::endl;
            }  
        }
        if (auto_exposure_ == 1 )
        {
            option_type = static_cast<rs2_option>(10);
            sensor.set_option(option_type, 1);
        }else
        {
            option_type = static_cast<rs2_option>(10);
            sensor.set_option(option_type, 0);
        }
        if (auto_Backlight_Compensation_ == 1 )
        {
            option_type = static_cast<rs2_option>(0);
            sensor.set_option(option_type, 1);
        }else
        {
            option_type = static_cast<rs2_option>(0);
            sensor.set_option(option_type, 0);
        }
        saveCameraFile();
    // }
    
}

void Camera_control::getparaCallback(const tku_msgs::camera &msg)
{
    brightness_         = float(msg.brightness);
    contrast_           = float(msg.contrast);
    saturation_         = float(msg.saturation);
    white_balance_      = float(msg.white_balance);
    auto_white_balance_ = msg.auto_white_balance;
    auto_exposure_       = msg.auto_exposure; 
    auto_Backlight_Compensation_ = msg.auto_Backlight_Compensation;

    SetCameraParameter();  
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "realsense_cam");
    ros::NodeHandle nh;
    // ros::NodeHandle privateNodeHandle;
    // RealSenseNodeFactory realSenseNodeFactory;
    // ros::NodeHandle nh = realSenseNodeFactory.getNodeHandle();
	// auto privateNh = realSenseNodeFactory.getPrivateNodeHandle();
    
    Camera_control camera_control(nh);
    
    //camera_control->print_device_information();
    camera_control.loadCameraFile();
    ros::Rate loop_rate(60);
    
    while (nh.ok())
    {
        //camera_control->print_separator();
        camera_control.sensor = camera_control.get_sensor_from_a_device();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}