#include "cArcballCamera.hpp"
#include <iostream>
#include <glm/gtc/matrix_transform.hpp>
#include <util/cGlmUtil.hpp>
#include <util/cJsonUtil.hpp>
#include <GLFW/glfw3.h>
// #include <glm/gtx/string_cast.hpp>

cArcballCamera::cArcballCamera(const std::string & conf) : cBaseCamera(conf)
{
	/*
				Y   front
				|  /
				| /
				/
				-------x
			  /
			 /
			Z=-front
	*/
	
	// parse json
	ParseConfig(conf);

	mCameraFocus = mCameraRestFocus;
	 
	Reload();
}

void cArcballCamera::Reload()
{
	mCameraFront = (mCameraFocus - mCameraPos).normalized();
	cBaseCamera::Reload();
}

void cArcballCamera::SetFocus(const tVector & focus)
{
	tVector move = mCameraFocus - mCameraPos;
	mCameraFocus = focus;
	mCameraPos = mCameraFocus - move;
	Reload();
}

void cArcballCamera::KeyEvent(int key, int scancode, int action, int mods)
{
	if(action == GLFW_PRESS || action == GLFW_REPEAT)
	{
		switch (key)
		{
		case GLFW_KEY_W:
			mCameraPos += 0.1 * (mCameraFocus - mCameraPos);
			break;
		case GLFW_KEY_S:
			mCameraPos -= 0.1 * (mCameraFocus - mCameraPos);
			// std::cout << mCameraPos.transpose() << std::endl;
			break;
		default:
			break;
		}
	}

	cBaseCamera::KeyEvent(key, scancode, action, mods);
}

void cArcballCamera::MouseMoveEvent(double xpos, double ypos)
{	
    if(mCameraActive == true && false == std::isnan(mCursorLastY) && false == std::isnan(mCursorLastX))
    {
        // std::cout <<"-----------------------\n";
		// std::cout << "arcball camera cursor = " << xpos << " " << ypos << std::endl;
		tVector front = mCameraFocus - mCameraPos;
		// std::cout <<"camera pos = " << mCameraPos.transpose() << std::endl;
		// std::cout <<"camera focus = " << mCameraFocus.transpose() << std::endl;
		// std::cout <<"camera up = " << mCameraUp.transpose() << std::endl;
        double move_vel = 5e-3;
        tVector move_vec = tVector(xpos - mCursorLastX, ypos - mCursorLastY, 0, 0) * move_vel;
        // std::cout <<"raw move vec = " << move_vec.transpose() << std::endl;
        move_vec[1] *=-1;
        // std::cout <<"move vec in tranditional RH frame = " << move_vec.transpose() << std::endl;
        move_vec = cMathUtil::RotMat(cMathUtil::AxisAngleToQuaternion(tVector(0, 0, 1, 0) * cMathUtil::Radians(90))) * move_vec; 
        // std::cout <<"move vec after 90 d= " << move_vec.transpose() << std::endl;
        move_vec = mViewTrans.transpose() * move_vec;
        // std::cout <<"move vec in world coor = " << move_vec.transpose() << std::endl;
        tMatrix rot_mat = cMathUtil::RotMat(cMathUtil::AxisAngleToQuaternion(move_vec));
        rot_mat.transposeInPlace();
        // std::cout <<"rot mat = " << rot_mat << std::endl;

        front = rot_mat * front;
        mCameraUp = rot_mat * mCameraUp;
        if(true == std::isnan(mCameraUp.maxCoeff()))
        {
            std::cout <<"move vec in world frame = " << move_vec.transpose() << std::endl;
            std::cout <<"rot mat = " << rot_mat.transpose() << std::endl;
            exit(1);
        }

		// update camera pos
		mCameraPos = mCameraFocus - front;
		mCameraFront = front.normalized();
    }

    cBaseCamera::MouseMoveEvent(xpos, ypos);

    Reload();
}

void cArcballCamera::MouseButtonEvent(int button, int action, int mods)
{
	cBaseCamera::MouseButtonEvent(button, action, mods);
}

void cArcballCamera::ScrollEvent(double offset)
{
	if(mCameraFixed) return;
	tVector diff_vec = mCameraFocus - mCameraPos;
	diff_vec *= (1 + 0.1 * offset);
	mCameraPos = mCameraFocus - diff_vec;
	Reload();
}

void cArcballCamera::ParseConfig(const std::string & conf)
{
	Json::Value root;
	cJsonUtil::ParseJson(conf, root);
	mCameraRestFocus = tVector(0, 0, 0, 1);
	for(int i=0; i< 3;i ++)
		mCameraRestFocus[i] = root["Camera"]["CameraFocus"][i].asDouble();
	// std::cout <<"[debug] arcball camera focus = " << mCameraFocus.transpose() << std::endl;
}

void cArcballCamera::Reset()
{
	mCameraFocus = mCameraRestFocus;
	cBaseCamera::Reset();
}