#include <util/cMathUtil.hpp>

enum eLightType{
    SQUARE,
    BALL
};

class cBaseMesh;
struct tRay;
struct tFace;
class cLight{
public:
    cLight(eLightType type, const tVector & rad);
    enum eLightType GetType();
    virtual void Sample_Li(const tVector & ref, tRay & wi_ray, double * pdf) = 0;
    virtual void GetDrawShape(std::vector<tFace> & face) const = 0;
    tVector GetRadiance(){return mRadiance;}
private:
    eLightType mType;
protected:
    std::vector<tFace> mLightFaces;
    const tVector mRadiance;
};

class cSquareLight: public cLight{
public:
    cSquareLight(const tVector light_pt[4], const tVector & rad);
    virtual void Sample_Li(const tVector & ref, tRay & wi_ray, double * pdf) override final;
    virtual void GetDrawShape(std::vector<tFace> & face) const override final;
private:
    tVector mLightPos[4];
    tVector mNormal;
    double mArea;
};

std::vector<std::shared_ptr<cLight>> BuildLight(const std::string & conf);