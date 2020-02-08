#include <util/cMathUtil.hpp>
#include <geometry/cBaseMesh.h>

class cBaseCamera;
class cPathTracer
{
public: 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    cPathTracer(const std::string &conf);
    void Init(std::shared_ptr<cBaseMesh> scene_mesh, std::shared_ptr<cBaseCamera> camera);
    void Update(std::vector<tLine> & lines, std::vector<tVertex> &pts);
    void GetRayLines(std::vector<tLine> &line);
    
private:
    std::shared_ptr<cBaseMesh> mSceneMesh;
    std::shared_ptr<cBaseCamera> mCamera;
    tVector mCameraPos;
    bool mAccelStructure;
    int mWidth, mHeight;
    double mFov, mNear;
    int mDivide;

    tVector * mScreen;
    tRay * mScreenRay;
    std::vector<tAABB> mAABBLst;

    // methods
    void ParseConfig(const std::string & conf);
    void BuildAccelStructure();
    void UpdatePrimaryRay();
    void RayCastPrimaryRay(std::vector<tLine> & lines, std::vector<tVertex> & pts)const;
    void RayCastSingleRay(const tRay & ray, tVector & pt)const;
    void DispatchFaceToAABB(const tFace * face);
};