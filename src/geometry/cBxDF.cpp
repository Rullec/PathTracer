#include "cBxDF.hpp"
#include <geometry/cBaseMesh.h>
#include <util/cGeoUtil.hpp>

cBxDF::cBxDF()
{
    mIsLight = false;
    mType = eBxDFType::INVALID;
}

// brdf
cBRDF::cBRDF(const tVector& ka, const tVector&  kd, const double& ni, const double& ns, const tVector& ks):
    mKa(ka), mKd(kd), mNi(ni), mNs(ns), mKs(ks)
{
    if(ka.norm() > 1e-5)
    {
        mIsLight = true;
    }
    // std::cout <<"[debug] cBSDF::cBSDF get ka = " << mKa.transpose() <<"\n"
    // << "kd = " << mKd.transpose() <<"\n"
    // << "ni = " << mNi <<"\n"
    // << "ns = " << mNs <<"\n"
    // << "ks = " << mKs.transpose() <<"\n";
}

tVector cBRDF::Sample_f()
{
    if(mIsLight) return tVector::Zero();

    return tVector::Zero();
}

double cBRDF::pdf(const tVector & wi, const tVector & wo)
{
    return 1.0;
}
/*
    wi: light to ref pt
    wo: ref pt to outgoing

    brdf = kd / pi + ks * (n + 2) / (2 * pi) * cos^n(alpha)
    alpha: the angle between ideal specular outgoing and true outgoing
*/
tVector cBRDF::evaluate(const tVector & wi, const tVector & wo, const tVector & normal)
{
    assert(cMathUtil::IsVector(wi) && cMathUtil::IsNormalized(wi));
    assert(cMathUtil::IsVector(wo) && cMathUtil::IsNormalized(wo));
    assert(cMathUtil::IsVector(normal) && cMathUtil::IsNormalized(normal));
    double cos_alpha = cGeoUtil::Reflect(normal, wi).dot(wo);
    // std::cout <<" mkd = " << mKd.transpose() / M_PI << std::endl;
    // return mKd / M_PI;
    return mKd / M_PI + mKs * (mNs + 2) / ( 2 * M_PI) * pow(cos_alpha, mNs);
}

// bsdf
cBSDF::cBSDF(const tVector& ka, const tVector&  kd, const double& ni, \
const double& ns, const tVector& ks, const tVector& tf):
    mKa(ka), mKd(kd), mNi(ni), mNs(ns), mKs(ks), mTf(tf)
{
    std::cout <<"[debug] cBSDF::cBSDF get ka = " << mKa.transpose() <<"\n"
    << "kd = " << mKd.transpose() <<"\n"
    << "ni = " << mNi <<"\n"
    << "ns = " << mNs <<"\n"
    << "ks = " << mKs.transpose() <<"\n"
    << "tf = " << mTf.transpose() <<"\n";

    if(mKa.norm() > 1e-5) mIsLight = true;
}

tVector cBSDF::Sample_f()
{
    if(mIsLight) return tVector::Zero();
    return tVector::Zero();
}

double cBSDF::pdf(const tVector & wi, const tVector & wo)
{
    return 1.0;
}

std::shared_ptr<cBxDF> BuildBxDF(tMaterial * material)
{
    if(nullptr == material)
    {
        std::cout <<"[error] BuildBxDF empty input\n";
        exit(1);
    }

    const tVector Ka = material->ambient;  // 环境光颜色, Ka
    const tVector & Kd = material->diffuse;  // 漫反射颜色, Kd
    const int & illum = material->illum;    // 种类
    const double & Ni = material->ior;      // 折射率光流密度
    const double & Ns = material->shininess;    // 高光指数
    const tVector & Ks = material->specular;// 高光项颜色
    const tVector & Tf = material->transmittance;    // 透射颜色

    std::shared_ptr<cBxDF> bxdf;
    // if(Tf.norm() > 1e-5)
    // {
    //     // build BSDF
    //     bxdf = std::shared_ptr<cBxDF>(new cBSDF(Ka, Kd, Ni, Ns, Ks, Tf));
    // }
    // else
    // {
    //     bxdf = std::shared_ptr<cBxDF>(new cBRDF(Ka, Kd, Ni, Ns, Ks));
    // }

    bxdf = std::shared_ptr<cBxDF>(new cBRDF(Ka, Kd, Ni, Ns, Ks));
    return bxdf;
}

tVector cBSDF::evaluate(const tVector & v1, const tVector & v2, const tVector & normal)
{
    return tVector::Zero();
}