#pragma once
#include <util/cMathUtil.hpp>
#include <geometry/cBaseMesh.h>

enum eBxDFType{
    INVALID, 
    BRDF,   // reflect
    // BTDF,   // transmit
    BSDF    // scatter
};

struct tMaterial;
class cLight;
class cAccelStruct;
// BSDF = BTDF + BRDF
class cBxDF{
public:
    cBxDF();
    // virtual double pdf(const tVector & wi, const tVector & wo) = 0;
    virtual tVector evaluate(const tVector & wi, const tVector & wo, const tVector & normal) = 0;
    virtual tVector Sample_Li(const std::vector<std::shared_ptr<cLight>> & lights, cAccelStruct * mAccelStruct, const tVector & ref_normal, const tVector & ref_point, const tRay & wo_ray);
    virtual tVector Sample_f(const tVector & ref_normal, const tVector & wo, tVector & wi_dir) = 0;
    eBxDFType GetType(){return mType;}
protected:
    eBxDFType mType;
    // bool mIsLight;
};

class cBRDF : public cBxDF
{
    // diffuse + specular
public:
    cBRDF(const tVector& ka, const tVector&  kd, const double& ns, const tVector& ks);
    // virtual double pdf(const tVector & wi, const tVector & wo) override;
    // virtual tVector Sample_Li(const std::vector<std::shared_ptr<cLight>> & lights, cAccelStruct * mAccelStruct, const tVector & ref_normal, const tVector & ref_point, const tRay & wo_ray) override;
    virtual tVector evaluate(const tVector & wi, const tVector & wo, const tVector & normal) override;
    virtual tVector Sample_f(const tVector & ref_normal, const tVector & wo, tVector & wi_dir) override;
    
protected:
    const tVector mKa, mKd, mKs;
    const double mNs;
};

class cBSDF: public cBxDF{
public:
    cBSDF(const tVector& ka, const tVector&  kd, const double& ni, const double& ns, const tVector& ks);
    virtual tVector Sample_f(const tVector & ref_normal, const tVector & wo, tVector & wi_dir) override;
    // virtual double pdf(const tVector & wi, const tVector &  wo) override;
    virtual tVector evaluate(const tVector & wi, const tVector & wo, const tVector & normal) override;
    // virtual tVector Sample_Li(const std::vector<std::shared_ptr<cLight>> & lights, cAccelStruct * mAccelStruct, const tVector & ref_normal, const tVector & ref_point, const tRay & wo_ray) override;
protected:
    const tVector mKa, mKd, mKs;
    const double  mNi, mNs;
    double R0;
};

std::shared_ptr<cBxDF> BuildBxDF(tMaterial * mat);