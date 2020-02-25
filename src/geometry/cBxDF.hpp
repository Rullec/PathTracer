#pragma once
#include <util/cMathUtil.hpp>
enum eBxDFType{
    INVALID, 
    BRDF,   // reflect
    BTDF,   // transmit
    BSDF    // scatter
};

struct tMaterial;
// BSDF = BTDF + BRDF
class cBxDF{
public:
    cBxDF();
    virtual tVector Sample_f() = 0;
    virtual double pdf(const tVector & wi, const tVector & wo) = 0;
    virtual tVector evaluate(const tVector & wi, const tVector & wo, const tVector & normal) = 0;
protected:
    eBxDFType mType;
    bool mIsLight;
};

class cBRDF : public cBxDF
{
    // diffuse + specular
public:
    cBRDF(const tVector& ka, const tVector&  kd, const double& ni, const double& ns, const tVector& ks);
    virtual tVector Sample_f() override;
    virtual double pdf(const tVector & wi, const tVector & wo) override;
    virtual tVector evaluate(const tVector & wi, const tVector & wo, const tVector & normal) override;
protected:
    const tVector mKa, mKd, mKs;
    const double  mNi, mNs;
};

class cBSDF: public cBxDF{
public:
    cBSDF(const tVector& ka, const tVector&  kd, const double& ni, const double& ns, const tVector& ks, const tVector& tf);
    virtual tVector Sample_f() override;
    virtual double pdf(const tVector & wi, const tVector &  wo) override;
    virtual tVector evaluate(const tVector & wi, const tVector & wo, const tVector & normal) override;
protected:
    const tVector mKa, mKd, mKs, mTf;
    const double  mNi, mNs;
};

std::shared_ptr<cBxDF> BuildBxDF(tMaterial * mat);