#include "cBaseCamera.hpp"
#include <util/cMathUtil.hpp>

class cFPSCamera : public cBaseCamera
{
public:
    cFPSCamera(const std::string &conf);

    virtual void MouseMoveEvent(double xpos, double ypos) override final;
    virtual void MouseButtonEvent(int button, int action,
                                  int mods) override final;
    virtual void KeyEvent(int key, int scancode, int action,
                          int mods) override final;
    virtual void ScrollEvent(double offset) override final;

protected:
    virtual void Reload() override final;
    virtual void Reset() override final;

private:
    void ParseConf(const std::string &conf);
};