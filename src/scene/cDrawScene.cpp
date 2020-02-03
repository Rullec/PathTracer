#include "cDrawScene.hpp"
#include <render/BuildRender.hpp>
#include <render/camera/BuildCamera.hpp>
#include <cstring>
#ifdef WIN32
#include <Windows.h>
#endif
#include <ctime>
#include <iostream>

extern GLFWwindow * gWindow;

// test
cDrawScene::cDrawScene(const std::string & config):cScene()
{
	// init value
	mDataReload = true;

	// build render
	BuildRender(config, mRender);
	if (mRender == nullptr)
	{
		std::cout << "[error] cDrawScene set up pixel render failed " << std::endl;
		exit(1);
	}

	// build camera, set camera
	mCamera = BuildCamera(config);
	mRender->SetCamera(mCamera);
}

cDrawScene::~cDrawScene()
{

}

void cDrawScene::Init()
{
	if (mSceneStatus != eSceneStatus::BeforeInit)
	{
		std::cout << "[error] cDrawScene::Init status error: " << cScene::mSceneStatusName[mSceneStatus] << std::endl;
		exit(1);
	}
	
	// init render
	mRender->Init();

	// change status
	mSceneStatus = eSceneStatus::InitSucc;
}

void cDrawScene::Update()
{
	if (mSceneStatus != eSceneStatus::InitSucc)
	{
		std::cout << "[error] cDrawScene Update: status error" << std::endl;
		exit(1);
	}

	if (mDataReload == true)
	{
		mRender->Clear();
		DrawScene();

		DrawAxis();
		mDataReload = false;
	}

	// draw
	mRender->Draw();
}

void cDrawScene::ParseConfig(const std::string & conf)
{

}


void cDrawScene::DrawScene()
{

}

void cDrawScene::DrawAxis()
{
	auto render = std::dynamic_pointer_cast<cBaseRender>(mRender);
	tEdge cur;
	cur.mOri = new tVertex(); cur.mDest = new tVertex();
	for (int i = 0; i < 3; i++)
	{
		tVector ori_pos = tVector::Zero(),
			dest_pos = tVector::Zero(),
			color = tVector::Zero();
		ori_pos[3] = 1;
		dest_pos[3] = 1;
		dest_pos[i] = 1000;
		color[i] = 1;
		color[3] = 1;
		
		// set up line
		cur.mOri->mPos = ori_pos;
		cur.mDest->mPos = dest_pos;
		cur.mOri->mColor = color;
		cur.mDest->mColor = color;

		render->AddLine(cur);
	}
}

void cDrawScene::KeyEvent(int key, int scancode, int action, int mods)
{
	mCamera->KeyEvent(key, scancode, action, mods);
	//mPicker->KeyEvent(key, scancode, action, mods);
}

void cDrawScene::MouseMoveEvent(double xpos, double ypos)
{
	mCamera->MouseMoveEvent(xpos, ypos);
}

void cDrawScene::MouseButtonEvent(int button, int action, int mods)
{
	mCamera->MouseButtonEvent(button, action, mods);
}

void cDrawScene::ScrollEvent(double offset)
{
	mCamera->ScrollEvent(offset);
}

bool cDrawScene::AddObjToScene(std::shared_ptr<cBaseMesh> obj)
{
	// std::cout << "[debug] cDrawScene Add obj" << std::endl;
	if(obj == nullptr) return false;

	mRender->AddMesh(obj);

	// const std::vector<tFace * > & faces = obj->GetFaceList();
	// for(auto & face : faces)
	// {
	// 	mRender->AddFace(face->mVertexPtrList);
	// }
	
	return true;
}