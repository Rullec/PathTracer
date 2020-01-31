#include "cDrawScene.hpp"
#include <render/BuildRender.hpp>
#include <cstring>
#ifdef WIN32
#include <Windows.h>
#endif
#include <ctime>
#include <iostream>
extern GLFWwindow * gWindow;

// test
#include <geometry/cMeshLoader.h>
cDrawScene::cDrawScene(const std::string & config):cScene()
{
	// init value
	mCurFrame = 0;
	mDataReload = true;
	glfwGetFramebufferSize(gWindow, &mCurFrameWidth, &mCurFrameHeight);
	mCameraActive = false;
	mCursorLastX = std::nan("");
	mCursorLastY = std::nan("");

	// build render
	BuildRender(config, mRender);
	if (mRender == nullptr)
	{
		std::cout << "[error] cDrawScene set up pixel render failed " << std::endl;
		exit(1);
	}

	// build camera, set camera
	mCamera = std::make_shared<cFocusCamera>(this->mCurFrameWidth, this->mCurFrameHeight);
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
	for (auto & face : mFaces)
	{
		mRender->AddPolygon(face);
	}
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
	//mPicker->KeyEvent(key, scancode, action, mods);
}

void cDrawScene::MouseMoveEvent(double xpos, double ypos)
{
	if (std::isnan(mCursorLastX) == true)
	{
		mCursorLastX = xpos;
		mCursorLastY = ypos;
		return;
	}

	if (mCameraActive)
	{
		tVector move_vec = tVector(xpos, ypos, 0, 1) - tVector(mCursorLastX, mCursorLastY, 0, 1);
		mCamera->Rotate(move_vec);
	}

	mCursorLastX = xpos;
	mCursorLastY = ypos;
}

void cDrawScene::MouseButtonEvent(int button, int action, int mods)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT)
	{
		if (action == GLFW_PRESS) mCameraActive = true;
		else
		{
			mCameraActive = false;
			mCursorLastY = mCursorLastX = std::nan("");
		}
	}
}

void cDrawScene::ScrollEvent(double offset)
{
	mCamera->Scroll(offset);
	//mDataReload = true;
}