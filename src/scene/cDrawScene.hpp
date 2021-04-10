#pragma once
#include <render/cBaseRender.hpp>
#include <render/camera/cBaseCamera.hpp>
#include <scene/cScene.hpp>

class cQuadViser;
struct tPolygon;
class cDrawScene : public cScene
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    cDrawScene(const std::string &config);
    ~cDrawScene();
    virtual void Init() override;
    virtual void Update() override;
    virtual void KeyEvent(int key, int scancode, int action, int mods) override;
    virtual void MouseMoveEvent(double xpos, double ypos) override;
    virtual void MouseButtonEvent(int button, int action, int mods) override;
    virtual void ScrollEvent(double offset) override;

protected:
    std::shared_ptr<cBaseRender> mRender;
    std::shared_ptr<cBaseCamera> mCamera;
    bool mDataReload;

    // rendering info
    virtual void DrawScene();
    bool AddObjToScene(std::shared_ptr<cBaseMesh> obj);
    void DrawAxis();
    void DrawGround();
    void ParseConfig(const std::string &conf);

private:
    bool mDrawAxis;
};