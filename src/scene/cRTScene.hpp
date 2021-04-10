#include "cDrawScene.hpp"

class cBaseMesh;
class cPathTracer;
struct tMeshParams;
class cRTScene : public cDrawScene
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    cRTScene(const std::string &config);

    virtual void Init() override final;
    virtual void Update() override final;
    virtual void KeyEvent(int key, int scancode, int action,
                          int mods) override final;
    virtual void MouseMoveEvent(double xpos, double ypos) override final;
    virtual void MouseButtonEvent(int button, int action,
                                  int mods) override final;
    virtual void ScrollEvent(double offset) override final;

protected:
    struct tMeshParams *mParams;
    std::shared_ptr<cBaseMesh> mModel;
    std::shared_ptr<cPathTracer> mTracer;

    // tool methods
    void LoadModel(const std::string &model_name, double scale);
    virtual void DrawScene() override final;
};