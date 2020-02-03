#include "cRTScene.hpp"
#include <util/cJsonUtil.hpp>
#include <geometry/cMeshLoader.h>

cRTScene::tParams::tParams()
{
    mModelName = "";
    mModelScale = 1.0;
}

cRTScene::cRTScene(const std::string & config) : cDrawScene(config)
{
    Json::Value root;
    cJsonUtil::ParseJson(config, root);
    Json::Value scene_json = root["Scene"];
    mParams.mModelName = scene_json["ModelName"].asString();
    mParams.mModelScale = scene_json["ObjScale"].asDouble();
}

void cRTScene::Init()
{
    cDrawScene::Init();

    // load model and draw model
    LoadModel(mParams.mModelName, mParams.mModelScale);

}

void cRTScene::Update()
{
    cDrawScene::Update();
}

void cRTScene::LoadModel(const std::string & model_name, double scale)
{
    // load obj
    mModel = cMeshLoader::Load(model_name, eMeshType::OBJ, scale);

    tVector up, low;
    mModel->GetBound(up, low);
    // std::cout <<"[debug] model up = " << up.transpose() << std::endl;
    // std::cout <<"[debug] model low = " << low.transpose() << std::endl;
}

void cRTScene::DrawScene()
{
    // add model to scene
    AddObjToScene(mModel);
}