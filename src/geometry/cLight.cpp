#include "cLight.hpp"
#include <geometry/cBaseMesh.h>
#include <util/cJsonUtil.hpp>
#include <util/cGeoUtil.hpp>

cLight::cLight(eLightType type, const tVector & v):mType(type), mRadiance(v)
{
    this->mLightFaces.clear();
}

eLightType cLight::GetType()
{
    return mType;
}


cSquareLight::cSquareLight(const tVector light_pt[4], const tVector & v):cLight(eLightType::SQUARE, v)
{
    // std::cout <<"[debug] init square light begin\n";
    tVector normal1 = cGeoUtil::CalcNormalFrom3Pts(light_pt[0], light_pt[1], light_pt[2]),
    normal2 = cGeoUtil::CalcNormalFrom3Pts(light_pt[1], light_pt[2], light_pt[3]);

    // std::cout << normal1.transpose() <<" " << normal2.transpose() << std::endl;
    assert(cMathUtil::IsSame(normal1, normal2, 2e-3) == true);
    // std::cout <<"[debug] square light: normal = " << normal1.transpose() << std::endl;

    mNormal = normal1;
    assert(cMathUtil::IsVector(mNormal));
    for(int i=0; i<4; i++)
    {
        mLightPos[i] = light_pt[i];
        // std::cout <<"[debug] square light = " << mLightPos[i].transpose() << std::endl;

        assert(cMathUtil::IsPoint(mLightPos[i]));
    } 
    mArea = (mLightPos[2] - mLightPos[1]).norm() * (mLightPos[1] - mLightPos[0]).norm();
    // std::cout <<"[debug] square light: area = " << mArea<< std::endl;

    // exit(1);
    // build light faces
    {
        tFace faces[2];
        for(int i=0; i< 3; i++)
        {
            faces[0].mVertexPtrList[i] = new tVertex();
            faces[0].mVertexPtrList[i]->mPos = mLightPos[i];
            faces[0].mVertexPtrList[i]->mColor = tVector(1, 1, 1, 0);
            // std::cout <<"add pos = " << faces[0].mVertexPtrList[i]->mPos.transpose() << std::endl;
        }
        mLightFaces.push_back(faces[0]);

        for(int i=0; i< 3; i++)
        {
            faces[1].mVertexPtrList[i] = new tVertex();
            faces[1].mVertexPtrList[i]->mPos = mLightPos[(i + 2) % 4];
            faces[1].mVertexPtrList[i]->mColor = tVector(1, 1, 1, 0);
            // std::cout <<"add pos = " << faces[1].mVertexPtrList[i]->mPos.transpose() << std::endl;
        }
        mLightFaces.push_back(faces[1]);
        
    }
}

// sample from light 
/*
    @Function: cSquareLight::Sample_Li
    @params: ref Type const tVector &, reference point in the scene surface, which will be lit by this light.
    @params: pdf Type double *, the probability of incident light,

Hint:
    light_ray: from light source to pt
    pdf = r^2 / (A * cos_theta_0)
*/
void cSquareLight::Sample_Li(const tVector & ref, tRay & wi_ray, double * pdf)
{
    double xi1 = drand48(), xi2 = drand48();
    tVector sample_pt = mLightPos[0] + (mLightPos[1] - mLightPos[0] ) * xi1;
    sample_pt += (mLightPos[2] - mLightPos[1]) * xi2;
    assert(cMathUtil::IsPoint(sample_pt));

    // normal test
    tVector ray_dir = (ref - sample_pt).normalized();
    assert(cMathUtil::IsVector(ray_dir));
    // {
    //     for(int i=0; i<4; i++) std::cout <<mLightPos[i].transpose() << std::endl;
    //     std::cout <<"light = " << ray_dir.transpose() << std::endl;
    //     std::cout <<"sample_pt = " << sample_pt.transpose() << std::endl;
    //     std::cout <<"ref = " << ref.transpose() << std::endl;
    //     exit(1);
    // }
    if(ray_dir.dot(mNormal) <= 0)
    {
        *pdf = 0;
        wi_ray.Init(tVector::Zero(), tVector::Zero());
        return;
    }
    else
    {
        // wi_ray: from light to reference point
        // assert(cMathUtil::IsNormalized(ray_dir));
        wi_ray.Init(sample_pt, ray_dir);
        *pdf = pow((sample_pt - ref).norm(), 2) / (mArea * ray_dir.dot(mNormal)) * (1.0 / (2 * M_PI));
        // std::cout <<"[debug] sample light succ: " << sample_pt.transpose() <<", pdf = " << *pdf << std::endl; 
    }
    // std::cout <<"\n[light] sample pt = " << sample_pt.transpose() << ", ref pt = " << ref.transpose() << std::endl;
    // std::cout <<"ray dir = " << ray_dir.transpose() << std::endl;
    // std::cout <<"pdf = " << *pdf << std::endl;
    // exit(1);
}

void cSquareLight::GetDrawShape(std::vector<tFace> & faces) const
{   
    for(int i=0; i<mLightFaces.size(); i++)
        faces.push_back(mLightFaces[i]);
}

/*
class cSphereLight: public cLight{
    cSphereLight(const tVector & center, double radius);
    virtual void Sample_Li(const tVector & ref, tRay & wi_ray, double * pdf) override final;
    virtual void GetDrawShape(std::vector<tFace> & face) const override final;
private:
    const tVector mCenter;
    const double mRadius;
}
*/

const int layers = 100;
const int samples = 100;
// create triangles for unit sphere
tVertex vertices[layers+1][samples+1];
cSphereLight::cSphereLight(const tVector & center, double radius, const tVector & radiance):mCenter(center), mRadius(radius), cLight(eLightType::SPHERE, radiance)
{
    std::cout <<"[debug] sphere light init = " << mCenter.transpose() <<" " << radius << std::endl;
    assert(cMathUtil::IsPoint(center));

    // init face
    {
        for(int l = 0; l <= layers; l++)
        for(int s = 0; s <= samples; s++)
        {
            // from up to down
            // double theta = M_PI;
            auto & cur_v = vertices[l][s];
            double theta = s * (2 * M_PI) / samples;
            double z = 1 - 2.0 * l / layers;
            double local_radius = std::sqrt(1.0 - z * z);
            cur_v.mPos.x() = std::cos(theta) * local_radius * radius + center[0];
            cur_v.mPos.y() = std::sin(theta) * local_radius * radius  + center[1];
            cur_v.mPos.z() = z * radius + center[2];
            cur_v.mNormal = cur_v.mPos.normalized();
            cur_v.mColor = tVector(0.85, 0.75, 0.65, 0);
            // std::cout << vertices[l][s].transpose() <<std::endl;
// std::cout <<  << std::endl;
        }

        tFace face;
        for(int l = 0; l < layers; l++)
        for(int s = 0; s < samples; s++)
        {
            tVertex & v1 = vertices[l][s],
            &v2 = vertices[l+1][s],
            &v3 = vertices[l+1][s+1],
            &v4 = vertices[l][s+1];

            face.mVertexPtrList[0] = &v1;
            face.mVertexPtrList[1] = &v2;
            face.mVertexPtrList[2] = &v3;
            this->mLightFaces.push_back(face);
            face.mVertexPtrList[0] = &v3;
            face.mVertexPtrList[1] = &v4;
            face.mVertexPtrList[2] = &v1;
            this->mLightFaces.push_back(face);
        }

    }
    // exit(1);
}

/*
    sample a point over a uniform light sphere
*/
void cSphereLight::Sample_Li(const tVector & ref, tRay & wi_ray, double * pdf)
{
    assert(cMathUtil::IsPoint(ref));

    // 1. calculate max_theta and sample a direction
    double dc = (ref - mCenter).norm(); // from ref pt to center pt
    assert(dc > mRadius);
    double max_theta = std::asin(mRadius / dc);
    tVector from_ref_to_light = (mCenter - ref).normalized();
    tVector sample_dir = cMathUtil::SampleCone(from_ref_to_light, max_theta, *pdf);
    assert(cMathUtil::IsVector(sample_dir));
    assert(cMathUtil::IsNormalized(sample_dir));

    // 2. calculate the intersection point between the light sphere and ref pt
    // ds = dc * cos(theta) - sqrt(r^2 - d_c^2 * sin^2(theta))
    double theta = std::acos(sample_dir.dot(from_ref_to_light));
    double ds = dc * std::cos(theta) - std::sqrt(mRadius * mRadius - dc * dc * pow(std::sin(theta), 2));
    tVector intersection_pt = ref + ds * sample_dir;
    // judge intersection pt is on the surface of this sphere
    assert(cMathUtil::IsPoint(intersection_pt));
    assert(std::fabs((intersection_pt - mCenter).norm() - mRadius) < 1e-6);

    // 3. set up incident ray wi:
    wi_ray.Init(ref, -from_ref_to_light);

    // // double xi1 = drand48(), xi2 = drand48();
    // tVector sample_pt = cMathUtil::SampleSphereUniform(*pdf) * mRadius + mCenter;
    // tVector light_normal = (sample_pt - mCenter).normalized();
    // assert(cMathUtil::IsPoint(sample_pt));
    // assert(cMathUtil::IsVector(light_normal));

    // // normal test
    // tVector ray_dir = (ref - sample_pt).normalized();
    // assert(cMathUtil::IsVector(ray_dir));

    // if(ray_dir.dot(light_normal) <= 0)
    // {
    //     *pdf = 0;
    //     wi_ray.Init(tVector::Zero(), tVector::Zero());
    //     return;
    // }
    // else
    // {
    //     // wi_ray: from light to reference point
    //     // assert(cMathUtil::IsNormalized(ray_dir));
    //     wi_ray.Init(sample_pt, ray_dir);
    //     *pdf = pow((sample_pt - ref).norm(), 2) / (ray_dir.dot(light_normal) * 4 * M_PI * pow(mRadius,2));
    //     // std::cout <<"[debug] sample light succ: " << sample_pt.transpose() <<", pdf = " << *pdf << std::endl; 
    // }
}

void cSphereLight::GetDrawShape(std::vector<tFace> & face) const
{
    // std::cout << "sphare light get draw shape \n";
    // exit(1);
    for(auto &s : this->mLightFaces) face.push_back(s);
}

// build light
std::vector<std::shared_ptr<cLight>> BuildLight(const std::string & conf)
{
    Json::Value root, scene, light_info_lst, light_type_lst, light_radiance_lst;
    if(false == cJsonUtil::ParseJson(conf, root))
    {
        std::cout <<"[error] BuildLight parse failed " << conf << std::endl;
        exit(1);
    }

    std::shared_ptr<cLight> light;
    double scale = root["Scene"]["ObjScale"].asDouble();
    scene = root["Light"];
    int light_num = scene["LightNum"].asInt();
    light_type_lst = scene["LightType"];
    light_info_lst = scene["LightInfo"];
    light_radiance_lst = scene["LightRadiance"];

    if(scale < 1e-5 || light_num == 0 || light_type_lst.isNull() || light_info_lst.isNull() || light_radiance_lst.isNull())
    {
        std::cout <<"[error] BuildLight light parse failed " << scale << light_num << light_info_lst.isNull() << light_type_lst.isNull() << light_radiance_lst.isNull() << std::endl;
        exit(1);
    }

    assert(light_info_lst.size() == light_num);
    assert(light_type_lst.size() == light_num);
    assert(light_radiance_lst.size() == light_num);
    std::vector<std::shared_ptr<cLight>> light_res_lst(light_num);
    for(int light_id=0; light_id<light_num; light_id++)
    {
        const std::string type = light_type_lst[light_id].asString();
        Json::Value light_info = light_info_lst[light_id];
        tVector light_radiance = tVector::Zero();
        // std::cout <<"----------------\n";
        for(int i=0; i<3; i++) light_radiance[i] = light_radiance_lst[light_id][i].asDouble();
        // std::cout <<"radiance = " << light_radiance.transpose() << std::endl;
        if(type == "square")
        {
            tVector square_pts[4] = { tVector::Zero(), tVector::Zero(), tVector::Zero(), tVector::Zero() };
            Json::Value sub_pt;
            for(int i=0; i<4; i++)
            {
                sub_pt = light_info[i];
                if(true == sub_pt.isNull())
                {
                    std::cout << "[error] BuildLight: sub point fetched fail\n";
                    exit(1);
                }

                for(int j=0; j<4; j++) 
                {
                    square_pts[i][j] = sub_pt[j].asDouble() * scale;
                }
                square_pts[i][3] /= scale;
            }
            light_res_lst[light_id] = std::shared_ptr<cLight>(new cSquareLight(square_pts, light_radiance));
            // std::cout << square_pts[0].transpose() << std::endl;

            // std::cout <<"[debug] light pos = " << square_pts[0].transpose() <<"\n" << 
            //     square_pts[1].transpose() << "\n" << 
            //     square_pts[2].transpose() << "\n" << 
            //     square_pts[3].transpose() << "\n";
        }
        else if(type == "sphere")
        {
            assert(light_info.isNull() == false);
            tVector sphere_center = tVector::Ones();
            double radius;
            for(int i=0; i<3; i++) sphere_center[i] = light_info[i].asDouble() * scale;
            radius = light_info[3].asDouble() * scale;
            light_res_lst[light_id] = std::shared_ptr<cLight>(new cSphereLight(sphere_center, radius, light_radiance));
        }
        else
        {
            std::cout <<"[error] BuildLight: Unsupported type " << type << std::endl;
            exit(1);
        }
    }
    // std::cout <<"no problem\n";
    // std::cout <<light_res_lst.size() << std::endl;
    // exit(1);

    return light_res_lst;
}