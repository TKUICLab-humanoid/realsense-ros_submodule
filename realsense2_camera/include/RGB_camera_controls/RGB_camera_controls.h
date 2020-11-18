//ifndef RGB_CAMERA_CONTROLS_H
//#define RGB_CAMERA_CONTROLS_H
#include <fstream>
#include <string>
#include <stdlib.h>
#include <string.h>
#include "tku_libs/TKU_tool.h"
//#include "../../../strategy/src/StrategyNameAndPath.h"
using namespace std;


struct CameraParameter
{
    bool  auto_exposure;
    bool  auto_white_balance;
    bool  auto_Backlight_Compensation;
    float  brightness;
    float  contrast;
    float  saturation;
    float  white_balance;
    string ParameterName;
};


class RGB_Camera_Controls
{
    public:
        RGB_Camera_Controls();
        ~RGB_Camera_Controls();
        void SaveCameraSetFile();
        void LoadCameraSetFile();
        ToolInstance *tool;

        CameraParameter* CameraParameterValue;
};
extern RGB_Camera_Controls* RGB_camera_controls;

//#endif 
