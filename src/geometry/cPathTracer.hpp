#include <util/cMathUtil.hpp>
#include <geometry/cBaseMesh.h>

class cBaseCamera;
class cPathTracer
{
public: 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    cPathTracer(const std::string &conf);
    void Init(std::shared_ptr<cBaseMesh> scene_mesh, std::shared_ptr<cBaseCamera> camera);
    void Process();
    void GetDrawResources(std::vector<tLine> & line, std::vector<tVertex> & pts);
    
private:
    std::shared_ptr<cObjMesh> mSceneMesh;
    std::shared_ptr<cBaseCamera> mCamera;

    // camera parameters
    int mWidth, mHeight;
    double mFov, mNear;

    // ray tracing configuration
    bool mAccelStructure;
    bool mRayDisplay;
    std::string mResultPath;
    int mDivide;
    int mMaxDepth;

    // storage
    tVector * mScreenPixel;
    tRay * mScreenRay;
    std::vector<tAABB> mAABBLst;
    std::vector<tLine> mDrawLines;
    std::vector<tVertex> mDrawPoints;

    // methods
    void ParseConfig(const std::string & conf);
    void BuildAccelStructure();
    void GenerateRay();
    void RayTracing();
    tVector RayCastSingleRay(const tRay & ray, tVector & pt, int depth)const;
    void OutputImage();
};