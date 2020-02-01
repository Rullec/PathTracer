#include "cDrawScene.hpp"

class cBaseMesh;

class cRTScene: public cDrawScene{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    struct tParams{
        std::string mModelName;
        tParams();
    };

    cRTScene(const std::string & config);

    void Init() override final;
    void Update() override final;

protected:
    struct tParams mParams;
    std::shared_ptr<cBaseMesh> mModel;

    // tool methods
    void LoadModel(const std::string & model_name);
    virtual void DrawScene() override final;
};