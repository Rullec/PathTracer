#include "cDrawScene.hpp"
#include "cRTScene.hpp"
#include "cScene.hpp"
#include <fstream>
#include <iostream>
#include <util/cJsonUtil.hpp>

enum eSceneType
{
    DrawScene,
    RTScene, // ray tracing scene
    NUM_SCENE_TYPE
};
const std::string gStrSceneType[eSceneType::NUM_SCENE_TYPE] = {"DrawScene",
                                                               "RTScene"};

std::shared_ptr<cScene> BuildScene(std::string path)
{
    Json::Value root;
    if (false == cJsonUtil::ParseJson(path, root))
    {
        std::cout << "[error] BuildScene: parse json error: " << path
                  << std::endl;
        exit(1);
    }

    std::shared_ptr<cScene> scene = nullptr;
    std::string scene_type = root["Scene"]["Type"].asString();
    // std::cout <<"[debug] scene type = " << scene_type<<std::endl;
    for (int i = 0; i < eSceneType::NUM_SCENE_TYPE; i++)
    {
        if (gStrSceneType[i] == scene_type)
        {
            eSceneType enum_scene_type = static_cast<eSceneType>(i);
            switch (enum_scene_type)
            {
            case eSceneType::DrawScene:
                scene = (std::shared_ptr<cScene>)(new cDrawScene(path));
                break;
            case eSceneType::RTScene:
                scene = (std::shared_ptr<cScene>)(new cRTScene(path));
                break;
            default:
                break;
            }
        }
    }

    if (scene == nullptr)
    {
        std::cout << "[error] BuildScene: Check scene type failed: "
                  << scene_type << std::endl;
        exit(1);
    }

    return scene;
}