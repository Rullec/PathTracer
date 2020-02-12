#include "cRTScene.hpp"
#include <util/cJsonUtil.hpp>
#include <geometry/cMeshLoader.h>
#include <geometry/cPathTracer.hpp>

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

    mTracer = std::shared_ptr<cPathTracer>(new cPathTracer(config));
}

void cRTScene::Init()
{
    cDrawScene::Init();

    // load model and draw model
    LoadModel(mParams.mModelName, mParams.mModelScale);

    // init path tracer
    mTracer->Init(mModel, mCamera);
}

void cRTScene::Update()
{
    // draw lines and set flat mDataReload
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

    // path tracer update
    // 1. generate primary rays
    // 2. ray cast
    std::vector<tLine> rays;
    std::vector<tVertex> pts;
    // mTracer->Update(rays, pts);
    mTracer->Process();
    mTracer->GetDrawResources(rays, pts);
    std::cout <<"get draw res = " << rays.size() <<" " << pts.size() << std::endl;

    for(auto & ray : rays)
        mRender->AddLine(ray);
    // std::cout <<"ray inter = " << pts.size() << std::endl;
    for(int i=0 ;i<pts.size(); i++)
    {
        mRender->AddPoint(pts[i]);
    }
}

void cRTScene::KeyEvent(int key, int scancode, int action, int mods)
{
    // mDataReload = true;
    cDrawScene::KeyEvent(key, scancode, action, mods);
	// mCamera->KeyEvent(key, scancode, action, mods);
	//mPicker->KeyEvent(key, scancode, action, mods);
}

void cRTScene::MouseMoveEvent(double xpos, double ypos)
{
    // mDataReload = true;
    cDrawScene::MouseMoveEvent(xpos, ypos);
	// mCamera->MouseMoveEvent(xpos, ypos);
}

void cRTScene::MouseButtonEvent(int button, int action, int mods)
{
    // mDataReload = true;
    cDrawScene::MouseButtonEvent(button, action, mods);
	// mCamera->MouseButtonEvent(button, action, mods);
}

void cRTScene::ScrollEvent(double offset)
{
    // mDataReload = true;
    cDrawScene::ScrollEvent(offset);
	// mCamera->ScrollEvent(offset);
}
