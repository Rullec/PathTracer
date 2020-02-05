#include "cBaseCamera.hpp"
#include <util/cJsonUtil.hpp>
#include <GLFW/glfw3.h>
#include <iostream>
#include <glm/gtc/matrix_transform.hpp>
#include <util/cGlmUtil.hpp>

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


tMatrix cBaseCamera::GetViewMat()
{
	return mViewTrans;
}

tMatrix cBaseCamera::GetProjMat()
{
	return mProjTrans;
}

tMatrix cBaseCamera::GetRenderMat()
{
	// rendermat = view * proj
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

void cBaseCamera::GetCameraScene(int & width, int & height, double & fov, double &near)
{
	width = mWindowWidth;
	height = mWindowHeight;
	fov = mFOV;
	near = mNear;
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
	mCameraRestFront = tVector(0, 0, -1, 0);

	// 	tVector mCameraPos, mCameraUp, mCameraFront, mCameraFocus;
	for(int i=0; i<3; i++)
	{
		mCameraRestPos[i] = camera_info["CameraPos"][i].asDouble();
		mCameraRestUp[i] = camera_info["CameraUp"][i].asDouble();
		mCameraRestFront[i] = camera_info["CameraFront"][i].asDouble();
	}
	mFOV = camera_info["CameraFov"].asDouble();
	mNear = camera_info["CameraNear"].asDouble();
	mFar = camera_info["CameraFar"].asDouble();
	// std::cout << mNear << mFar << std::endl;
	// exit(1);
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
    if(action == GLFW_PRESS || action == GLFW_REPEAT)
    {
		switch (key)
		{
		case GLFW_KEY_R:
			Reset();
			break;
		
		default:
			break;
		}
	}

	Reload();
}

void cBaseCamera::ScrollEvent(double offset)
{
	
}

void cBaseCamera::Reload()
{
	{
        glm::vec3 pos = cGlmUtil::tVectorToGlmVector3(mCameraPos),
                front = cGlmUtil::tVectorToGlmVector3(mCameraFront),
                up = cGlmUtil::tVectorToGlmVector3(mCameraUp);
        glm::mat4 view_mat = glm::lookAt(pos, pos + front, up);
        mViewTrans = cGlmUtil::GlmMatixTotMatrix(view_mat);
    }
    // mViewTrans = tMatrix::Identity();

    // calculate projection 
    {
        // glm::mat4 projection = glm::perspective(glm::radians(fov), 800.0f / 600.0f, 0.1f, 100.0f); 
        // mProjTrans = cGlmUtil::GlmMatixTotMatrix(projection);
        glm::mat4 projection = glm::perspective(glm::radians(mFOV), mWindowWidth / mWindowHeight, mNear, mFar); 
        mProjTrans = cGlmUtil::GlmMatixTotMatrix(projection);
    }
    // mProjTrans = tMatrix::Identity();
    mRenderMat =  mProjTrans * mViewTrans;
}

void cBaseCamera::Reset()
{
	mCameraPos = mCameraRestPos;
	mCameraUp = mCameraRestUp;
	mCameraFront = mCameraRestFront;
}
