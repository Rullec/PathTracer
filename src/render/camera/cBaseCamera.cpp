#include "cBaseCamera.hpp"
#include <util/cJsonUtil.hpp>
#include <GLFW/glfw3.h>
#include <iostream>

cBaseCamera::cBaseCamera(const std::string & conf)
{
    ParseConf(conf);

	mCameraPos = mCameraRestPos;
	mCameraUp = mCameraRestUp;
	mCameraFront = mCameraRestFront;

	mCameraActive = false;
	mCursorLastX = std::nan("");
	mCursorLastY = std::nan("");
}

void cBaseCamera::SetCameraPos(const tVector & pos)
{
    mCameraPos = pos;
    Reload();
}

void cBaseCamera::SetCameraUp(const tVector & up)
{
    mCameraUp = up;
    Reload();
}

tMatrix cBaseCamera::GetRenderMat()
{
    return mRenderMat;
}

tVector cBaseCamera::GetCameraPos()
{
    return mCameraPos;
}

tVector cBaseCamera::GetCameraFront()
{
    return mCameraFront;
}

void cBaseCamera::ParseConf(const std::string & conf)
{
     Json::Value root;
	cJsonUtil::ParseJson(conf, root);

	Json::Value window_info = root["MainWindowInfo"];
	mWindowWidth = window_info["Width"].asFloat();
	mWindowHeight = window_info["Height"].asFloat();

	Json::Value camera_info = root["Camera"];

	mCameraFixed = camera_info["Fixed"].asBool();
	
	mCameraRestPos = tVector(0, 0, 1, 1);
	mCameraRestUp = tVector(0 ,1, 0, 0);
	mCameraRestFront = tVector(0, 0, -1, 1);

	// 	tVector mCameraPos, mCameraUp, mCameraFront, mCameraFocus;
	for(int i=0; i<3; i++)
	{
		mCameraRestPos[i] = camera_info["CameraPos"][i].asDouble();
		mCameraRestUp[i] = camera_info["CameraUp"][i].asDouble();
		mCameraRestFront[i] = camera_info["CameraFront"][i].asDouble();
	}

	// std::cout <<"[debug] camera pos = " << mCameraRestPos.transpose() << std::endl;
	// std::cout <<"[debug] camera front = " << mCameraRestFront.transpose() << std::endl;
	// std::cout <<"[debug] camera up = " << mCameraRestUp.transpose() << std::endl;
	// std::cout <<"[debug] camera fixed = " << mCameraFixed << std::endl;
}


void cBaseCamera::MouseMoveEvent(double xpos, double ypos)
{
	mCursorLastX = xpos;
	mCursorLastY = ypos;
}

void cBaseCamera::MouseButtonEvent(int button, int action, int mods)
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

void cBaseCamera::KeyEvent(int key, int scancode, int action, int mods)
{

}

void cBaseCamera::ScrollEvent(double offset)
{
	
}