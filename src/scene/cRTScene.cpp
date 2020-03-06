#include "cRTScene.hpp"
#include <util/cJsonUtil.hpp>
#include <geometry/cMeshLoader.h>
#include <tracing/cPathTracer.hpp>

cRTScene::cRTScene(const std::string & config) : cDrawScene(config)
{ 
    // std::cout <<"rt scene begin\n";
    Json::Value root;
    cJsonUtil::ParseJson(config, root);
    Json::Value scene_json = root["Scene"];
    assert(scene_json.isNull() == false);
    mParams = new struct tMeshParams();
    mParams->name = scene_json["ModelName"].asString();
    mParams->scale = scene_json["WorldScale"].asDouble();
    mParams->type = eMeshType::OBJ;
    mParams->shape_analysis = scene_json["ModelEnableShapeAnalysis"].asBool();
    mParams->build_edge_info = scene_json["ModelBuildEdge"].asBool();
    mParams->displacement = tVector::Zero();
    assert(scene_json["ModelDisplacement"].isNull() == false);
    for(int i=0; i<3; i++) mParams->displacement[i] = scene_json["ModelDisplacement"][i].asDouble();
    // std::cout <<"obj scale = " << mParams->scale <<", displace = " << mParams->displacement.transpose() << std::endl;
    // std::cout << mParams->build_edge_info << " " << mParams->shape_analysis << std::endl;
    // exit(1);
    mTracer = std::shared_ptr<cPathTracer>(new cPathTracer(config));
}

void cRTScene::Init()
{
    cDrawScene::Init();

    // load model and draw model
    mModel = cMeshLoader::Load(*mParams);

    // init path tracer
    mTracer->Init(mModel, mCamera);
}

void cRTScene::Update()
{
    // draw lines and set flat mDataReload
    cDrawScene::Update();
}

// #include <util/cTimeUtil.hpp>
void cRTScene::DrawScene()
{
    // add model to scene
    AddObjToScene(mModel);
    // path tracer update
    // 1. generate primary rays
    // 2. ray cast
    std::vector<tLine> rays;
    std::vector<tVertex> pts;
    std::vector<tFace> faces;
    // mTracer->Update(rays, pts);
    mTracer->Process();
    mTracer->GetDrawResources(rays, pts, faces);
    // std::cout <<"get draw res = " << rays.size() <<" " << pts.size() << std::endl;

    for(auto & ray : rays)
        mRender->AddLine(ray);
    // std::cout <<"ray inter = " << pts.size() << std::endl;
    for(int i=0 ;i<pts.size(); i++)
    {
        mRender->AddPoint(pts[i]);
    }
    for(auto & f : faces)
        mRender->AddFace(f.mVertexPtrList);
    cDrawScene::DrawScene();
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
