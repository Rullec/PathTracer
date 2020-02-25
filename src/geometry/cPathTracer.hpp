#include <util/cMathUtil.hpp>
#include <geometry/cBaseMesh.h>

class cBaseCamera;
class cLight;
class cBxDF;
class cPathTracer
{
public: 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    cPathTracer(const std::string &conf);
    void Init(std::shared_ptr<cBaseMesh> scene_mesh, std::shared_ptr<cBaseCamera> camera);
    void Process();
    void GetDrawResources(std::vector<tLine> & line, std::vector<tVertex> & pts, std::vector<tFace> & faces);
    
private:
    std::shared_ptr<cObjMesh> mSceneMesh;
    std::vector<std::shared_ptr<cLight>> mLight;
    std::vector<std::shared_ptr<cBxDF>> mBxDF;
    std::shared_ptr<cBaseCamera> mCamera;
    
    // camera parameters
    int mWidth, mHeight;
    double mFov, mNear;

    // ray tracing configuration
    bool mAccelStructure;
    bool mRayDisplay;
    bool mOpenResult;
    bool mEnableIndrectLight;
    bool mDrawLight;
    std::string mResultPath;
    int mDivide;
    int mMaxDepth;
    int mSamples;
    
    // storage
    const std::string mConfPath;
    tVector * mScreenPixel;
    tRay * mScreenRay;
    std::vector<tAABB> mAABBLst;
    std::vector<tLine> mDrawLines;
    std::vector<tVertex> mDrawPoints;

    // methods
    void ParseConfig(const std::string & conf);
    void BuildAccelStructure();
    void InitBxDF();

    void GenerateRay();
    void RayTracing();

    tVector RayTracePrimaryRay(const tRay & ray, int id) const;
    static tFace * RayCast(const tRay & ray, tVector & pt, const bool mAccelStructure,const std::shared_ptr<cObjMesh> mSceneMesh,const std::vector<tAABB> & mAABBLst);
    static bool VisTestForLight(const tVector & p1, const tVector & p2,const bool mAccelStructure,const std::shared_ptr<cObjMesh> mSceneMesh,const std::vector<tAABB> & mAABBLst);
    void OutputImage();
};