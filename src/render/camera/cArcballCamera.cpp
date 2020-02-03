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
	
	this->mCameraFocus = tVector(0, 0, 0, 1);

	// parse json
	ParseConfig(conf);
	Reload();
}

void cArcballCamera::Reload()
{
	// let: camera front is Z, camera up is Y, and ZxY = X
	// 0. calculates mCameraFront = mCameraFocus - mCameraPos
	{
		mCameraFront = (mCameraFocus - mCameraPos).normalized();
	}

	// 1. Viewpoint coordinate
	//std::cout << "*************" << std::endl;
	{

		// std::cout <<"camera pos = " << mCameraPos.transpose() << std::endl;
		// std::cout <<"camera up = " << mCameraUp.transpose() << std::endl;
		// std::cout <<"";

		// glm计算view trans
		{
			glm::vec3 eye, center, up;
			eye = cGlmUtil::tVectorToGlmVector3(mCameraPos);
			center = cGlmUtil::tVectorToGlmVector3(mCameraFocus);
			up = cGlmUtil::tVectorToGlmVector3(mCameraUp);
			glm::mat4 view = glm::lookAt(eye, center, up);
			mViewTrans = cGlmUtil::GlmMatixTotMatrix(view);
		}

		// 自行计算view trans
		// {
		// 	tVector X = mCameraUp.cross3(-mCameraFront) / (mCameraUp.cross3(-mCameraFront).norm());
		// 	tVector Z = -mCameraFront;
		// 	tVector Y = X.cross3(mCameraFront);
		// 	tMatrix Right = tMatrix::Identity();
		// 	Right.block(0, 0, 1, 3) = X.segment(0, 3).transpose();
		// 	Right.block(1, 0, 1, 3) = Y.segment(0, 3).transpose();
		// 	Right.block(2, 0, 1, 3) = Z.segment(0, 3).transpose();
	
		// 	Right.block(0, 3, 4, 1) = Right * (-mCameraPos);
		// 	Right(3, 3) = 1;
		// 	// std::cout <<"glm view mat = \n " << mViewTrans << std::endl;
		// 	// std::cout <<"self view mat = \n " << Right << std::endl;
		// 	mViewTrans = Right;
			
		// 	// std::cout <<"X = " << X.transpose() << std::endl;
		// 	// std::cout <<"Y = " << Y.transpose() << std::endl;
		// 	// std::cout <<"Z = " << Z.transpose() << std::endl;
		// 	// std::cout <<"View trans = \n " << Right << std::endl;
		// }
	}

	// 2. Project transform
	{
		float fov = 45.0;
        glm::mat4 projection = glm::perspective(glm::radians(fov), mWindowWidth / mWindowHeight, 0.1f, 100.0f); 
        mProjTrans = cGlmUtil::GlmMatixTotMatrix(projection);
	}

	mRenderMat = mProjTrans * mViewTrans;
	// mRenderTrans = mViewTrans;

	// std::cout <<"1 0 0 = " << (mRenderTrans * tVector(1, 0, 0, 0)).transpose() << std::endl;
	// std::cout <<"0 1 0 = " << (mRenderTrans * tVector(0, 1, 0, 0)).transpose() << std::endl;
	// std::cout <<"0 0 1 = " << (mRenderTrans * tVector(0, 0, 1, 0)).transpose() << std::endl;
	// std::cout << "view mat = " << mViewTrans << std::endl;
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
		double move_vel = 1e-3;
		switch (key)
		{
		case GLFW_KEY_W:
			mCameraPos += 0.1 * (mCameraFocus - mCameraPos);
			break;
		case GLFW_KEY_S:
			mCameraPos -= 0.1 * (mCameraFocus - mCameraPos);
			break;
		default:
			break;
		}
		Reload();
	}
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
	mCameraFocus = tVector(0, 0, 0, 1);
	for(int i=0; i< 3;i ++)
		mCameraFocus[i] = root["Camera"]["CameraFocus"][i].asDouble();
	std::cout <<"[debug] arcball camera focus = " << mCameraFocus.transpose() << std::endl;
}