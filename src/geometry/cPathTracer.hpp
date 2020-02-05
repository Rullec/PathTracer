#include <util/cMathUtil.hpp>

class cBaseMesh;
class cBaseCamera;

struct tRay{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    tVector mOri;
    tVector mDir;
    tRay(): mOri(tVector::Zero()), mDir(tVector::Zero()) {};
    tRay(const tVector & ori, const tVector & dir):mOri(ori), mDir(dir){};
};

class tLine;
class cPathTracer
{
public: 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    cPathTracer(const std::string &conf);
    void Init(std::shared_ptr<cBaseMesh> scene_mesh, std::shared_ptr<cBaseCamera> camera);
    void Update();
    void GetRayLines(std::vector<tLine> &line);
    
private:
    std::shared_ptr<cBaseMesh> mSceneMesh;
    std::shared_ptr<cBaseCamera> mCamera;
    tVector mCameraPos;
    int mWidth, mHeight;
    double mFov, mNear;

    tVector * mScreen;
    tRay * mScreenRay;

    // methods
    void ParseConfig(const std::string & conf);
    void UpdatePrimaryRay();
};