#include "cRTScene.hpp"
#include <util/cJsonUtil.hpp>
#include <geometry/cMeshLoader.h>

cRTScene::tParams::tParams()
{
    mModelName = "";
}

cRTScene::cRTScene(const std::string & config) : cDrawScene(config)
{
    Json::Value root;
    cJsonUtil::ParseJson(config, root);
    Json::Value scene_json = root["Scene"];
    mParams.mModelName = scene_json["ModelName"].asString();
}

void cRTScene::Init()
{
    cDrawScene::Init();

    // load model and draw model
    LoadModel(mParams.mModelName);

}

void cRTScene::Update()
{
    cDrawScene::Update();
}

void cRTScene::LoadModel(const std::string & model_name)
{
    // load obj
    mModel = cMeshLoader::Load(model_name, eMeshType::OBJ);
    mModel->PrintInfo();
}

void cRTScene::DrawScene()
{
    // add model to scene
    AddObjToScene(mModel);
}