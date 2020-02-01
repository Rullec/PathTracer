#include "cArcballCamera.hpp"
#include "cFPSCamera.hpp"
#include "cBaseCamera.hpp"
#include <util/cJsonUtil.hpp>

std::shared_ptr<cBaseCamera> BuildCamera(const std::string & conf)
{
    std::shared_ptr<cBaseCamera> camera;

    Json::Value root;
    cJsonUtil::ParseJson(conf, root);
    Json::Value camera_json = root["Camera"];
    if(camera_json.isNull() == true)
    {
        std::cout <<"[error] BuildCamera: conf null\n";
        exit(1);
    }

    const std::string type_str = camera_json["Type"].asString();
    for(int i=0; i<eCameraType::NUM_CAMERA_TYPE; i++)
    {
        if(type_str != gStrCameraType[i]) continue;

        eCameraType type = static_cast<eCameraType>(i);
        switch (i)
        {
        case eCameraType::ARCBALL:
            camera = std::shared_ptr<cArcballCamera>(new cArcballCamera(conf));
            break;
        case eCameraType::FPS : 
            camera = std::shared_ptr<cFPSCamera>(new cFPSCamera(conf));
            break;
        default:
            break;
        }
    }

    if(nullptr == camera)
    {
        std::cout <<"[error] BuildCamera failed to init camera type " << type_str << std::endl;
        exit(1);
    }
    return camera;
}