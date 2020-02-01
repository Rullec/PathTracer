#pragma once
#include <util/cMathUtil.hpp>
#include <string>

enum eCameraType {
	ARCBALL,
	FPS,
	NUM_CAMERA_TYPE,
};

const std::string gStrCameraType[eCameraType::NUM_CAMERA_TYPE] = {
	"Arcball",
	"FPS"
};

class cBaseCamera{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    cBaseCamera(const std::string & conf);

	// set & get method
	void SetCameraPos(const tVector &);
	void SetCameraUp(const tVector &);

	tMatrix GetRenderMat();
	tVector GetCameraPos();
	tVector GetCameraFront();

	virtual void MouseMoveEvent(double xpos, double ypos);
	virtual void MouseButtonEvent(int button, int action, int mods);
	virtual void KeyEvent(int key, int scancode, int action, int mods);
	virtual void ScrollEvent(double offset); 

protected:
    tVector mCameraPos, mCameraUp, mCameraFront;
	tMatrix mViewTrans, mProjTrans, mRenderMat;

    tVector mCameraRestPos, mCameraRestUp, mCameraRestFront;
	
	bool mCameraFixed;
	float mFOV;
	float mWindowHeight, mWindowWidth;

	// camera control info
	bool mCameraActive;		// record camera status value for rotating smoothly
	double mCursorLastX, mCursorLastY;

	// recompute the key matrix if something changed
	virtual void Reload() = 0;

private:
	void ParseConf(const std::string & conf);
};