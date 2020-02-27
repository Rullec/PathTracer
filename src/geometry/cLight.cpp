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
        *pdf = pow((sample_pt - ref).norm(), 2) / (mArea * ray_dir.dot(mNormal)) / (2 * M_PI);
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
    std::vector<std::shared_ptr<cLight>> light_res_lst(light_num);
    for(int light_id=0; light_id<light_num; light_id++)
    {
        const std::string type = light_type_lst[light_id].asString();
        Json::Value light_info = light_info_lst[light_id];
        tVector light_radiance = tVector::Zero();
        for(int i=0; i<3; i++) light_radiance[i] = light_radiance_lst[i].asDouble();

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

            // std::cout <<"[debug] light pos = " << square_pts[0].transpose() <<"\n" << 
            //     square_pts[1].transpose() << "\n" << 
            //     square_pts[2].transpose() << "\n" << 
            //     square_pts[3].transpose() << "\n";
        }
        else if(type == "ball")
        {
            
        }
        else
        {
            std::cout <<"[error] BuildLight: Unsupported type " << type << std::endl;
            exit(1);
        }
    }

    return light_res_lst;
}