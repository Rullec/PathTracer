//#include "cFocusCamera.h"
//#include <GL/glfw3.h>
//#include <iostream>
//
//cFocusCamera::cFocusCamera(int width, int height):mWindowHeight(height), mWindowWidth(width)
//{
//	/*
//				Y   front
//				|  /
//				| /
//				/
//				-------x
//			  /
//			 /
//			Z=-front
//	*/
//	assert(mWindowHeight != 0);
//	assert(mWindowWidth != 0);
//
//	// control value init
//	mEnableNDCTrans = true;	// �رչ淶�豸����ϵ(NDC)�任
//	mEnablePerspProj = false;	// �ر�͸��ͶӰ, ѡ������ͶӰ
//
//	// recompute
//	Reset();
//}
//
//void cFocusCamera::Reset()
//{
//	mCameraPos = tVector(0, 0, 5, 1);
//	mCameraFocus = tVector(0, 0, 0, 1);
//	mCameraFront = (mCameraFocus - mCameraPos).normalized();
//	mCameraUp = tVector(0, 1, 0, 0);
//	
//	mRenderTrans = mViewTrans = mProjTrans = mDeviceTrans = tMatrix::Identity();
//	Reload();
//}
//
//void cFocusCamera::Reload()
//{
//	// let: camera front is Z, camera up is Y, and ZxY = X
//	// 0. calculates mCameraFront = mCameraFocus - mCameraPos
//	{
//		mCameraFront = (mCameraFocus - mCameraPos).normalized();
//	}
//
//	// 1. Viewpoint coordinate
//	//std::cout << "*************" << std::endl;
//	//{
//	//	//std::cout << "camera pos = " << mCameraPos.transpose() << std::endl;
//	//	//std::cout << "camera front = " << mCameraFront.transpose() << std::endl;
//	//	//std::cout << "camera up = " << mCameraUp.transpose() << std::endl;
//	//	tVector X = mCameraUp.cross3(-mCameraFront) / (mCameraUp.cross3(-mCameraFront).norm());
//	//	tVector Z = -mCameraFront;
//	//	tVector Y = X.cross3(mCameraFront);
//	//	tMatrix Right = tMatrix::Identity();
//	//	Right.block(0, 0, 4, 1) = X;
//	//	Right.block(0, 1, 4, 1) = Y;
//	//	Right.block(0, 2, 4, 1) = Z;
//	//	
//	//	Right.block(0, 3, 4, 1) = Right.transpose() * (-mCameraPos);
//	//	Right(3, 3) = 1;
//	//	mViewTrans = Right;
//	//}
//
//	//// 2. Project transform
//	//if(mEnablePerspProj)
//	//{
//	//	mProjTrans = tMatrix::Identity();
//	//	//mProjTrans(0, 0) = 1;
//	//	//mProjTrans(2, 2) = 0;
//
//	//	// scale by focus pos dist
//	//	mProjTrans /= (mCameraFocus - mCameraPos).norm();
//	//}
//	//
//	//// 3. Device transform
//	//{
//	//	mDeviceTrans = tMatrix::Identity();
//	//	//mDeviceTrans(0, 0) *= mWindowHeight * 1.0 / mWindowWidth;
//	//}
//
//	//mRenderTrans = mDeviceTrans * mProjTrans * mViewTrans;
//
//	{
//		tVector X = mCameraUp.cross3(-mCameraFront) / (mCameraUp.cross3(-mCameraFront).norm());
//		tVector Z = -mCameraFront;
//		tVector Y = X.cross3(mCameraFront);
//		tMatrix Right = tMatrix::Identity();
//		Right.block(0, 0, 1, 3) = X.segment(0, 3).transpose();
//		Right.block(1, 0, 1, 3) = Y.segment(0, 3).transpose();
//		Right.block(2, 0, 1, 3) = Z.segment(0, 3).transpose();
//
//		Right.block(0, 3, 4, 1) = Right * (-mCameraPos);
//		Right(3, 3) = 1;
//		mViewTrans = Right;
//	}
//
//	// 2. Project transform
//	if (mEnablePerspProj)
//	{
//		mProjTrans = tMatrix::Identity();
//		double width = 100, height = 100;
//		double depth_near = -0.1;
//		double depth_far = -50;
//		mProjTrans(0, 0) = 2.0 / width;
//		mProjTrans(1, 1) = 2.0 / height;
//		mProjTrans(2, 2) = 1.0 / (depth_near - depth_far);
//		mProjTrans(2, 3) = -depth_far / (depth_far - depth_far);
//
//	}
//	else
//	{
//		
//	}
//
//	// 3. Device transform
//	if(mEnableNDCTrans)
//	{
//		mDeviceTrans = tMatrix::Identity();
//		std::cout << mWindowHeight * 1.0 / mWindowWidth << std::endl;
//		mDeviceTrans(0, 0) *= mWindowHeight * 1.0 / mWindowWidth;
//		//mDeviceTrans = tMatrix::Identity();
//	}
//
//	mRenderTrans = mDeviceTrans * mProjTrans * mViewTrans;
//}
//
//void cFocusCamera::SetCameraPos(const tVector &res)
//{
//	mCameraPos = res;
//	Reload();
//}
//
//void cFocusCamera::SetCameraUp(const tVector &res)
//{
//	mCameraUp = res;
//	mCameraUp.normalize();
//	Reload();
//}
//
////void cFocusCamera::SetCameraFront(const tVector &res)
////{
////	mCameraFront = res;
////	mCameraFront.normalize();
////	Reload();
////}
//
//void cFocusCamera::SetWindowShape(int width, int height)
//{
//	mWindowHeight = height;
//	mWindowWidth = width;
//	Reload();
//}
//
//void cFocusCamera::SetFocus(const tVector & focus)
//{
//	tVector move = mCameraFocus - mCameraPos;
//	mCameraFocus = focus;
//	mCameraPos = mCameraFocus - move;
//	Reload();
//}
//
//tMatrix cFocusCamera::GetRenderMat()
//{
//	return mRenderTrans;
//}
//
//tVector cFocusCamera::GetCameraPos()
//{
//	return mCameraPos;
//}
//
//tVector cFocusCamera::GetCameraFront()
//{
//	return mCameraFront;
//}
//
//void cFocusCamera::Scroll(double offset)
//{
//	//std::cout << "void cFocusCamera::Scroll(double offset)\b";
//	tVector diff_vec = mCameraFocus - mCameraPos;
//	diff_vec *= (1 + 0.1 * offset);
//	mCameraPos = mCameraFocus - diff_vec;
//	
//	Reload();
//	//std::cout << "mat = " << mRenderTrans << std::endl;
//}
//
//void cFocusCamera::Rotate(tVector move_vec)
//{
//	// move vec in screen coordinate
//	/*
//		1. convet it to world coordinate
//		2. axis angle generate 2 rot
//		3. rot camera
//	*/
//	tMatrix rot_mat = tMatrix::Identity();
//	{
//		double move_vel = 1e-3;
//		double x_offset = move_vec[1], y_offset = move_vec[0];
//		x_offset *= move_vel, y_offset *= move_vel;
//		// to world frame
//		tVector x_offset_vec = tVector(1, 0, 0, 0) * x_offset;
//		tVector y_offset_vec = tVector(0, 1, 0, 0) * y_offset;
//		x_offset_vec = mViewTrans.transpose() * x_offset_vec;
//		y_offset_vec = mViewTrans.transpose() * y_offset_vec;
//
//		tQuaternion x_rot = cMathUtil::AxisAngleToQuaternion(x_offset_vec);
//		tQuaternion y_rot = cMathUtil::AxisAngleToQuaternion(y_offset_vec);
//		rot_mat = cMathUtil::RotMat(y_rot) * cMathUtil::RotMat(x_rot);
//	}
//
//	// change camera pos
//	{
//		tVector diff_vec = mCameraFocus - mCameraPos;
//		diff_vec = rot_mat * diff_vec;
//		mCameraPos = mCameraFocus - diff_vec;
//		mCameraFront = diff_vec.normalized();
//	}
//
//	// change camera up vector
//	{
//		mCameraUp = rot_mat * mCameraUp;
//	}
//
//	Reload();
//}
#include "cFocusCamera.h"
#include <GLFW/glfw3.h>
#include <iostream>
#include <glm/gtc/matrix_transform.hpp>
#include <util/cGlmUtil.hpp>

cFocusCamera::cFocusCamera(int width, int height) :mWindowHeight(height), mWindowWidth(width)
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
	Reset();
}

void cFocusCamera::Reset()
{
	mCameraPos = tVector(0, 0, 50, 1);
	mCameraFocus = tVector(0, 0, 0, 1);
	mCameraFront = (mCameraFocus - mCameraPos).normalized();
	mCameraUp = tVector(0, 1, 0, 0);

	mRenderTrans = mViewTrans = mProjTrans = mDeviceTrans = tMatrix::Identity();
	Reload();
}

void cFocusCamera::Reload()
{
	// let: camera front is Z, camera up is Y, and ZxY = X
	// 0. calculates mCameraFront = mCameraFocus - mCameraPos
	{
		mCameraFront = (mCameraFocus - mCameraPos).normalized();
	}

	// 1. Viewpoint coordinate
	//std::cout << "*************" << std::endl;
	{
		glm::vec3 eye, center, up;
		eye = cGlmUtil::tVectorToGlmVector3(mCameraPos);
		center = cGlmUtil::tVectorToGlmVector3(mCameraFocus);
		up = cGlmUtil::tVectorToGlmVector3(mCameraUp);
		glm::mat4 view = glm::lookAt(eye, center, up);
		mViewTrans = cGlmUtil::GlmMatixTotMatrix(view);
		//std::cout << "view mat = " << mViewTrans << std::endl;
	}

	// 2. Project transform
	{
		glm::mat4 pers = glm::perspective(glm::radians(90.0f), static_cast<float>(mWindowWidth * 1.0 / mWindowHeight), 0.01f, 1000.f);
		mProjTrans = cGlmUtil::GlmMatixTotMatrix(pers);
		//std::cout << "proj trans = " << mProjTrans << std::endl;
	}

	// 3. Device transform
	//{
	//	mDeviceTrans = tMatrix::Identity();
	//	mDeviceTrans(0, 0) *= mWindowHeight * 1.0 / mWindowWidth;
	//}

	mRenderTrans = mDeviceTrans * mProjTrans * mViewTrans;
	//std::cout << "res = " << mRenderTrans << std::endl;
}

void cFocusCamera::SetCameraPos(const tVector &res)
{
	mCameraPos = res;
	Reload();
}

void cFocusCamera::SetCameraUp(const tVector &res)
{
	mCameraUp = res;
	mCameraUp.normalize();
	Reload();
}

//void cFocusCamera::SetCameraFront(const tVector &res)
//{
//	mCameraFront = res;
//	mCameraFront.normalize();
//	Reload();
//}

void cFocusCamera::SetWindowShape(int width, int height)
{
	mWindowHeight = height;
	mWindowWidth = width;
	Reload();
}

void cFocusCamera::SetFocus(const tVector & focus)
{
	tVector move = mCameraFocus - mCameraPos;
	mCameraFocus = focus;
	mCameraPos = mCameraFocus - move;
	Reload();
}

tMatrix cFocusCamera::GetRenderMat()
{
	return mRenderTrans;
}

tVector cFocusCamera::GetCameraPos()
{
	return mCameraPos;
}

tVector cFocusCamera::GetCameraFront()
{
	return mCameraFront;
}

void cFocusCamera::Scroll(double offset)
{
	//std::cout << "void cFocusCamera::Scroll(double offset)\b";
	tVector diff_vec = mCameraFocus - mCameraPos;
	diff_vec *= (1 + 0.1 * offset);
	mCameraPos = mCameraFocus - diff_vec;
	Reload();
	//std::cout << "mat = " << mRenderTrans << std::endl;
}

void cFocusCamera::Rotate(tVector move_vec)
{
	// move vec in screen coordinate
	/*
		1. convet it to world coordinate
		2. axis angle generate 2 rot
		3. rot camera
	*/
	tMatrix rot_mat = tMatrix::Identity();
	{
		double move_vel = 1e-3;
		double x_offset = move_vec[1], y_offset = move_vec[0];
		x_offset *= move_vel, y_offset *= move_vel;
		// to world frame
		tVector x_offset_vec = tVector(1, 0, 0, 0) * x_offset;
		tVector y_offset_vec = tVector(0, 1, 0, 0) * y_offset;
		x_offset_vec = mViewTrans.transpose() * x_offset_vec;
		y_offset_vec = mViewTrans.transpose() * y_offset_vec;

		tQuaternion x_rot = cMathUtil::AxisAngleToQuaternion(x_offset_vec);
		tQuaternion y_rot = cMathUtil::AxisAngleToQuaternion(y_offset_vec);
		rot_mat = cMathUtil::RotMat(y_rot) * cMathUtil::RotMat(x_rot);
	}

	// change camera pos
	{
		tVector diff_vec = mCameraFocus - mCameraPos;
		diff_vec = rot_mat * diff_vec;
		mCameraPos = mCameraFocus - diff_vec;
		mCameraFront = diff_vec.normalized();
	}

	// change camera up vector
	{
		mCameraUp = rot_mat * mCameraUp;
	}

	Reload();
	//std::cout << "mat = " << mRenderTrans << std::endl;
}
