#include "RGB_camera_controls/RGB_camera_controls.h"

RGB_Camera_Controls* RGB_camera_controls = new RGB_Camera_Controls();

RGB_Camera_Controls::RGB_Camera_Controls()
{
    CameraParameterValue = new CameraParameter;
    CameraParameterValue->auto_exposure = 0;
    CameraParameterValue->auto_white_balance = 0;
    CameraParameterValue->auto_Backlight_Compensation = 0;
    CameraParameterValue->brightness = 0;
    CameraParameterValue->contrast = 0;
    CameraParameterValue->saturation = 0;
    CameraParameterValue->white_balance = 0;
    CameraParameterValue->ParameterName = "[Camera Set Parameter]";   
    tool = ToolInstance::getInstance();
}

RGB_Camera_Controls::~RGB_Camera_Controls()
{
    delete CameraParameterValue;
}

void RGB_Camera_Controls::SaveCameraSetFile()
{
    char path[200];
    printf("%s",path);
    std::string PATH = tool->getPackagePath("strategy");
    strcpy(path, PATH.c_str());
    strcat(path, "/CameraSet.ini");
    try
    {
//       ofstream OutFile(sFileName.c_str());
        ofstream OutFile(path);
        OutFile << CameraParameterValue->ParameterName;
        OutFile << "\n";
        OutFile << "brightness = ";
        OutFile << CameraParameterValue->brightness;
        OutFile << "\n";
        OutFile << "contrast = ";
        OutFile << CameraParameterValue->contrast;
        OutFile << "\n";
        OutFile << "saturation = ";
        OutFile << CameraParameterValue->saturation;
        OutFile << "\n";
        OutFile << "white_balance = ";
        OutFile << CameraParameterValue->white_balance;
        OutFile << "\n";
        OutFile << "auto_white_balance = ";
        OutFile << CameraParameterValue->auto_white_balance;
        OutFile << "\n";
        OutFile << "auto_exposure = ";
        OutFile << CameraParameterValue->auto_exposure;
        OutFile << "\n";
        OutFile << "auto_Backlight_Compensation = ";
        OutFile << CameraParameterValue->auto_Backlight_Compensation;
        OutFile << "\n";

        OutFile.close();
    }
    catch( exception e )
    {
    }
}

void RGB_Camera_Controls::LoadCameraSetFile()
{
    fstream fin;
    char line[100]; 
    char path[200];
    std::string PATH = tool->getPackagePath("strategy");
    strcpy(path, PATH.c_str());
    strcat(path, "/CameraSet.ini");

    fin.open(path, ios::in);
    //fin.open(("../../Parameter/Color_Model_Data/ColorModelData.ini"), ios::in);
    try
    {
        fin.getline(line,sizeof(line),'\n');
        CameraParameterValue->brightness = tool->readvalue(fin, "brightness", 0);
        CameraParameterValue->contrast = tool->readvalue(fin, "contrast", 0);
        CameraParameterValue->saturation = tool->readvalue(fin, "saturation", 0);
        CameraParameterValue->white_balance = tool->readvalue(fin, "white_balance", 0);
        CameraParameterValue->auto_white_balance = tool->readvalue(fin, "auto_white_balance", 0);
        CameraParameterValue->auto_exposure = tool->readvalue(fin, "auto_exposure", 0);
        CameraParameterValue->auto_Backlight_Compensation = tool->readvalue(fin, "auto_Backlight_Compensation", 0);
        fin.close();
    }
    catch(exception e)
    {
    }
}


