#pragma once
#include <string>
#include <util/cMathUtil.hpp>

enum eCameraType
{
    ARCBALL,
    FPS,
    NUM_CAMERA_TYPE,
};

const std::string gStrCameraType[eCameraType::NUM_CAMERA_TYPE] = {"Arcball",
                                                                  "FPS"};

class cBaseCamera
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    cBaseCamera(const std::string &conf);

    // set & get method
    void SetCameraPos(const tVector &);
    void SetCameraUp(const tVector &);

    tMatrix GetViewMat();
    tMatrix GetProjMat();
    tMatrix GetRenderMat();
    tVector GetCameraPos();
    tVector GetCameraFront();
    void GetCameraScene(int &width, int &height, double &fov, double &near);
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
    float mNear;
    float mFar;
    // camera control info
    bool mCameraActive; // record camera status value for rotating smoothly
    double mCursorLastX, mCursorLastY;

    // recompute the key matrix if something changed
    virtual void Reload();
    virtual void Reset();

private:
    void ParseConf(const std::string &conf);
};