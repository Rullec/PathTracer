#pragma once
#include "cBaseCamera.hpp"

class cArcballCamera : public cBaseCamera
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    cArcballCamera(const std::string &conf);

    // set & get methods

    void SetFocus(const tVector &focus);

    // key event
    virtual void KeyEvent(int key, int scancode, int action,
                          int mods) override final;
    virtual void MouseMoveEvent(double xpos, double ypos) override final;
    virtual void MouseButtonEvent(int button, int action,
                                  int mods) override final;
    virtual void ScrollEvent(double offset) override final;

protected:
    tVector mCameraFocus;
    tVector mCameraRestFocus;

    void ParseConfig(const std::string &conf);

    void Scroll(double offset);
    virtual void Reload() override final;
    virtual void Reset() override final;
};