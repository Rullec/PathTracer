#include <util/cMathUtil.hpp>

class cBaseMesh;
class cBaseCamera;
class tLine;

struct tRay{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
public:
    tRay(): mOri(tVector::Zero()), mDir(tVector::Zero()) 
    {

    };
    tRay(const tVector & ori, const tVector & dir){Init(ori, dir);};
    void Init(const tVector & ori, const tVector & dir)
    {
        mOri = ori;
        mDir = dir;
        mInvDir = tVector(1.0/ mDir[0], 1.0/ mDir[1], 1.0/ mDir[2], 0);
        for(int i=0; i<3; i++) sign[i] = (mInvDir[i] < 0);
    }
    tVector GetOri() const{return mOri;}
    tVector GetDir() const{return mDir;}
    tVector GetInvDir() const {return mInvDir;}
    const bool * GetSign() const{return sign;}

private:
    tVector mOri;
    tVector mDir;
    bool sign[3];
    tVector mInvDir;
};

struct tAABB{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Eigen::Vector2d bound[3];
    std::vector<int> mFaceId;
    tAABB()
    {
        bound[0] = bound[1] = bound[2] = Eigen::Vector2d::Zero();
        mFaceId.clear();
    };
    bool intersect(tRay & ray);
};

class tLine;
class tVertex;
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

    tVector * mScreen;
    tRay * mScreenRay;
    std::vector<tAABB> mAABBLst;

    // methods
    void ParseConfig(const std::string & conf);
    void BuildAccelStructure();
    void UpdatePrimaryRay();
    void RayCastPrimaryRay(std::vector<tLine> & lines, std::vector<tVertex> & pts);
    void RayCastSingleRay(const tRay & ray, tVector & pt);
};