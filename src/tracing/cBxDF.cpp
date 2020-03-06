#include "cBxDF.hpp"
#include <geometry/cBaseMesh.h>
#include <geometry/cAccelStruct.hpp>
#include <util/cGeoUtil.hpp>
#include "cLight.hpp"

cBxDF::cBxDF()
{
    // mIsLight = false;
    mType = eBxDFType::INVALID;
}

/*
  wo_ray: from remote point to this reference point, the direction is opposite to a normal definition.
*/
tVector cBxDF::Sample_Li(const std::vector<std::shared_ptr<cLight>> & lights, cAccelStruct * mAccelStruct, const tVector & ref_normal, const tVector & ref_pt, const tRay & wo_ray)
{
    // std::cout <<"sample li \n";
    tRay wi_ray;
    double pdf;
    tVector direct_color = tVector::Zero();
    for(auto & cur_light : lights)
    {
        tVector Li = cur_light->GetRadiance();
        // wi_ray: from light source to pt
        // pdf = r^2 / (A * cos_theta_0), if illegal, pdf = 0
        cur_light->Sample_Li(ref_pt, wi_ray, &pdf);

        // intersect with myself?
        if(pdf < 1e-10) continue;
        if(false == mAccelStruct->VisTest(wi_ray.GetOri(), ref_pt))
            continue;

        // L_i * cos(theta_i) * cos(theta_0) * A / r^2
        // = L_i * cos(theta_i) / p(w)
        assert(cMathUtil::IsVector(wi_ray.GetDir()));
        assert(cMathUtil::IsNormalized(wi_ray.GetDir()));
        double cos_theta_normal_wi = ref_normal.dot(-wi_ray.GetDir());
        if(cos_theta_normal_wi < 0) continue;

        tVector brdf_value = this->evaluate(wi_ray.GetDir(), -wo_ray.GetDir(), ref_normal);/* * M_PI * 2;*/
        direct_color += cMathUtil::WiseProduct(Li * cos_theta_normal_wi / pdf, brdf_value);
        assert(direct_color.minCoeff() > -1e-6);
        
    }
    return direct_color;
}

// brdf
cBRDF::cBRDF(const tVector& ka, const tVector&  kd, const double& ns, const tVector& ks):
    mKa(ka), mKd(kd), mNs(ns), mKs(ks)
{
    mType = eBxDFType::BRDF;
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

/*
    @Function: Sample_f
        sample the (Phong) lobe 
    @params: ref_normal Type const tVector &, the normal in this reflection point (aka ref point)
    @params: wo Type const tVector &, the outgoing light in this point, which is ref_pt->outside, opposite to the direction of primary ray
    @params: wi_dir Type tVector &, this value will be CHANGED, it means the sampled income light direction which is outside -> this ref point.
*/
tVector cBRDF::Sample_f(const tVector & ref_normal, const tVector & wo, tVector & wi_dir)
{
    double pdf;
    wi_dir = cMathUtil::SampleHemiSphereUniform(ref_normal, pdf);
    // wi_ray.Init(ref_pt, -wi_dir);   // wi_ray from light src to ref_pt
    return evaluate(-wi_dir, wo, ref_normal) * ref_normal.dot(wi_dir) / pdf;
}

// bsdf
cBSDF::cBSDF(const tVector& ka, const tVector&  kd, const double& ni, \
const double& ns, const tVector& ks):
    mKa(ka), mKd(kd), mNi(ni), mNs(ns), mKs(ks)
{
    // std::cout <<"[debug] cBSDF::cBSDF get ka = " << mKa.transpose() <<"\n"
    // << "kd = " << mKd.transpose() <<"\n"
    // << "ni = " << mNi <<"\n"
    // << "ns = " << mNs <<"\n"
    // << "ks = " << mKs.transpose() <<"\n";
    mType = eBxDFType::BSDF;

    R0 = pow((1.0 - ni) / (1.0 + ni), 2);
    std::cout <<"R0 = " << R0 << std::endl;
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

    // std::cout <<"Ni = " << Ni << std::endl;
    std::shared_ptr<cBxDF> bxdf;
    
    if(cMathUtil::IsSame(Ni, 1.0))
    {
        bxdf = std::shared_ptr<cBxDF>(new cBRDF(Ka, Kd, Ns, Ks));
        // std::cout <<"brdf\n";
    }
    else
    {
        // build BSDF
        bxdf = std::shared_ptr<cBxDF>(new cBSDF(Ka, Kd, Ni, Ns, Ks));
        // std::cout <<"bsdf\n";
    }

    // bxdf = std::shared_ptr<cBxDF>(new cBRDF(Ka, Kd, Ns, Ks));
    return bxdf;
}

// when bsdf receive wo, it should in the whole sphere but no semi-sphere...
/*
    wi: light to ref pt
    wo: ref pt to outgoing

    brdf = kd / pi + ks * (n + 2) / (2 * pi) * cos^n(alpha)
    alpha: the angle between ideal specular outgoing and true outgoing
*/
tVector cBSDF::evaluate(const tVector & wi, const tVector & wo, const tVector & normal)
{

    assert(cMathUtil::IsVector(wi) && cMathUtil::IsNormalized(wi));
    assert(cMathUtil::IsVector(wo) && cMathUtil::IsNormalized(wo));
    assert(cMathUtil::IsVector(normal) && cMathUtil::IsNormalized(normal));

    tVector value = tVector::Zero();
    tVector ideal_wo = tVector::Zero();
    ideal_wo = cGeoUtil::Refract(normal, wi, 1.0 / mNi);
    if(ideal_wo.hasNaN() == true)
    {
        ideal_wo = cGeoUtil::Reflect(normal, wi);
    }

    double cos_theta = ideal_wo.dot(wo);
    if(cos_theta < 0) cos_theta = 0;
    const int coef = 1;
    value = tVector::Ones() / (4 * M_PI) * pow(cos_theta, coef);
    // value = tVector::Ones() * (coef + 1) / (4 * M_PI) * pow(cos_theta, coef);

    assert(value.hasNaN() == false);
    return value;
}

/*
    wo: outgoing light, ref_pt -> remote_pt
    wi_oppo: incoming light oppo, ref_pt -> remote_pt
    @return: bsdf value 
*/
tVector cBSDF::Sample_f(const tVector & ref_normal, const tVector & wo, tVector & wi_oppo)
{
    double pdf = 1.0 / ( 4 * M_PI);
    double R = R0 + (1 - R0) * pow(1 - std::fabs(ref_normal.dot(-wo)), 5);
    double seed = drand48();
    tVector bsdf_value = tVector::Zero();
    if(seed < R)
    {
        // 反射
        wi_oppo = cGeoUtil::Reflect(ref_normal, -wo);

    }
    else
    {
        // 折射
        wi_oppo = cGeoUtil::Refract(ref_normal, -wo, 1.0 / 1.5);
        if(wi_oppo.hasNaN() == true)
        {
            wi_oppo = cGeoUtil::Reflect(ref_normal, -wo);
        }

    }
    double cos_theta = 1.0;
    bsdf_value = tVector::Ones() * pdf * cos_theta;
    // tVector indirect_coef = bsdf_value / (pdf) * std::fabs(wi_oppo.dot(ref_normal));
    tVector indirect_coef = bsdf_value / (pdf);
    // for(int i=0; i<3; i++) wi_oppo[i] += (drand48() - 0.5) * 0.1;
    // wi_oppo.normalize();

    
    // bsdf_value = evaluate(-wi_oppo, wo, ref_normal);
    // assert(bsdf_value.minCoeff() > -1e-6);

    // tVector indirect_coef = bsdf_value / (pdf) * std::fabs(wi_oppo.dot(ref_normal));
    // if(indirect_coef.minCoeff() < -1e-6)
    // {
    //     std::cout << indirect_coef << std::endl;
    //     exit(1);
    // }
    assert(indirect_coef.hasNaN() == false);
    return indirect_coef;
}