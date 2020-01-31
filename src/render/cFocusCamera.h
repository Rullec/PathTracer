#pragma once
#include <util/cMathUtil.hpp>

class cFocusCamera 
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	cFocusCamera(int width, int height);
	void Reset();

	// set & get method
	void SetCameraPos(const tVector &);
	void SetCameraUp(const tVector &);
	void SetWindowShape(int width, int height);
	void SetFocus(const tVector &);
	tMatrix GetRenderMat();
	tVector GetCameraPos();
	tVector GetCameraFront();

	// key event
	void Scroll(double offset);
	void Rotate(tVector move_vec);

protected:
	tVector mCameraPos, mCameraUp, mCameraFront, mCameraFocus;
	tMatrix mViewTrans, mProjTrans, mDeviceTrans, mRenderTrans;
	int mWindowWidth, mWindowHeight;

	// boolean control values
	bool mEnableNDCTrans;
	bool mEnablePerspProj;

	void Reload();
};