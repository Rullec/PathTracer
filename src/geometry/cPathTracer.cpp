#define ENABLE_OPENMP
#include "cPathTracer.hpp"
#include <geometry/cBaseMesh.h>
#include <render/camera/cBaseCamera.hpp>
#include <util/cTimeUtil.hpp>
#include <util/cFileUtil.h>
#include <util/cJsonUtil.hpp>
#include <util/cGeoUtil.hpp>
#include <geometry/cLight.hpp>
#include <geometry/cBxDF.hpp>
#include <geometry/cAccelStruct.hpp>
#include <omp.h>
// #define DEBUG_MODE


cPathTracer::cPathTracer(const std::string &conf):mConfPath(conf)
{
    ParseConfig(conf);
}

void cPathTracer::Init(std::shared_ptr<cBaseMesh> scene_mesh, std::shared_ptr<cBaseCamera> camera)
{
    if(scene_mesh == nullptr || camera == nullptr)
    {
        std::cout <<"[error] cPathTracer Init nullptr input!\n";
        exit(1);
    }

    mSceneMesh = std::dynamic_pointer_cast<cObjMesh>(scene_mesh); 
    if(mSceneMesh == nullptr)
    {
        std::cout <<"[error] cPathTracer::Init: No obj mesh\n";
        exit(1);
    }

    mCamera = camera;

    // std::cout <<"[debug] cPathTracer::Init begin\n";
    mCamera->GetCameraScene(mWidth, mHeight, mFov, mNear);

    // std::cout <<"[debug] cPathTracer::Init width, height = " << mWidth <<" " << mHeight << ", fov = " << mFov << ", near = " << mNear << std::endl;
    mScreenPixel = new tVector[mWidth * mHeight];
    mScreenRay = new tRay[mWidth * mHeight];

    // init accel structure
    InitAccelStruct();    

    // init light
    mLight = BuildLight(mConfPath);

    // init BxDF
    InitBxDF();

    // init draw resources
    mDrawLines.clear();
    mDrawPoints.clear();
}

void RayToLine(const tRay & ray, tLine & line)
{
    line.mOri = ray.GetOri();
    line.mDest = ray.GetDir() * 1e4 + line.mOri;
}

void cPathTracer::Process()
{
    if(mCamera == nullptr || mSceneMesh == nullptr)
    {
        std::cout <<"[error] cPathTracer::Process failed\n" << std::endl;
        exit(1);
    }

    // 测试
    GenerateRay();

    // std::cout <<"[debug] begin ray cast primary rays\n";
    RayTracing();
}

void cPathTracer::InitAccelStruct()
{
    // create AABB and divide the space
    if(nullptr == mSceneMesh)
    {
        std::cout <<" scene null failed\n";
        exit(1);
    }

    if(mAccelStruct == nullptr)
    {
        std::cout <<"[error] cPathTracer::InitAccelStruct empty ptr\n";
        exit(1);
    }
    mAccelStruct->Init(std::static_pointer_cast<cBaseMesh>(mSceneMesh));
}

void cPathTracer::InitBxDF()
{
    std::shared_ptr<cObjMesh> obj_mesh = std::dynamic_pointer_cast<cObjMesh>(mSceneMesh);
    if(obj_mesh == nullptr)
    {
        std::cout <<"[error] cPathTracer::BuildBxDF failed, empty mesh!" << std::endl;
        exit(1);
    }

    mBxDF.clear();
    for(int i=0; i<obj_mesh->GetMaterialNum(); i++)
    {
        mBxDF.push_back(BuildBxDF(obj_mesh->GetMaterial(i)));
    }
    std::cout <<"[cPathTracer] Init bxdf succ " << mBxDF.size() << std::endl;
}

void cPathTracer::GenerateRay()
{
    /*
        Update Primary Ray: https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-generating-camera-rays/generating-camera-rays
        1. from raster to NDC
            NDC_x in [0, 1] = (pixel_x + 0.5) / width  
            NDC_y in [0, 1] = (pixel_y + 0.5) / height 
        2. from NDC to screen space (2D)
                (screen coordinate: the origin located in the center of screen, y up, x right)
            screen_x in [-1, 1] = (2 NDC_x - 1)
            screen_y in [-1, 1] = (1 - 2 NDC_y)
        3. fron screen to camera space (3D)
            camera_x = screen_x * width / height * tan(a/2)
            camera_y = screen_y * tan(a/2)
            camera_z = -near
        4. from camera to world (3D)
            world = View^{-1} * camera
    */
//    cTimeUtil::Begin();
   tMatrix mat;
   {
       // shape the conversion mat
        tVector test = tVector(0 ,mHeight/2, 0, 1);
        tMatrix mat1 = tMatrix::Identity();
        mat1(0, 0) = 1.0 / mWidth;
        mat1(0, 3) = 0.5 / mWidth;
        mat1(1, 1) = 1.0 / mHeight;
        mat1(1, 3) = 0.5 / mHeight;
        // std::cout <<"after 1, vec = " << (test = mat1 * test).transpose() << std::endl;

        tMatrix mat2 = tMatrix::Identity();
        mat2(0, 0) = 2;
        mat2(0, 3) = -1;
        mat2(1, 1) = -2;
        mat2(1, 3) = 1;
        // std::cout <<"after 2, vec = " << (test = mat2 * test).transpose() << std::endl;
        
        // pos = mat2 * pos;
        tMatrix mat3 = tMatrix::Identity();
        mat3(0, 0) = mWidth / mHeight * std::tan(cMathUtil::Radians(mFov) / 2) * mNear;
        mat3(1, 1) = std::tan(cMathUtil::Radians(mFov) / 2) * mNear;
        mat3(2, 2) = 0, mat3(2, 3) = -mNear;
        // std::cout <<"after 3, vec = " << (test = mat3 * test).transpose() << std::endl;
       
        // std::cout << "mat 3 = " << mat3 << std::endl;
        // exit(1);
        // pos = mat3 * pos;
        tMatrix mat4 = mCamera->GetViewMat().inverse();
        // std::cout <<"after 4, vec = " << (test = mat4 * test).transpose() << std::endl;
        // std::cout <<"dir = " <<  (test - mCamera->GetCameraPos()).normalized().transpose() << std::endl;
        mat = mat4 * mat3 * mat2 * mat1;
        // exit(1);
    }
   tVector camera_pos = mCamera->GetCameraPos();
   double x_max = -1, x_min = 1, y_max = -1, y_min = 1;
    for(int y = 0; y < mHeight; y ++ )
    {
        for(int x = 0 ; x < mWidth; x++)
        {
            // [row, col]
            int access_id = y * mWidth + x;
            tVector pos = mat * tVector(x, y, 1, 1);
            tVector dir = (pos - camera_pos).normalized();
            // std::cout <<"dir = " << dir.transpose() << std::endl;
            mScreenRay[access_id].Init(camera_pos, dir);
            // if(dir[0] > x_max) x_max = dir[0];
            // if(dir[0] < x_min) x_min = dir[0];
            // if(dir[1] > y_max) y_max = dir[1];
            // if(dir[1] < y_min) y_min = dir[1];
            // if(dir[0] < -0.97)
            // {
            //     std::cout <<"x < -0.97" << std::endl;
            //     std::cout << "it is = " << x <<" " << y << std::endl;
            //     // std::cout <<"dir = " << 
            // }
            // std::cout << (pos - camera_pos).normalized().transpose() << std::endl;
        }
    }
    // std::cout <<" x from " << x_min <<" to " << x_max << std::endl;
    // std::cout <<" y from " << y_min <<" to " << y_max << std::endl;
    // exit(1);
    // cTimeUtil::End();
}


void cPathTracer::ParseConfig(const std::string & conf)
{
    // std::cout <<"[debug] cPathTracer::ParseConfig " << conf << std::endl;
    Json::Value root;
    cJsonUtil::ParseJson(conf, root);
    Json::Value path_tracer_json = root["PathTracer"];
    if(path_tracer_json.isNull() == true)
    {
        std::cout <<"[error] ParseConfig EnableAccel failed\n";
        exit(1);
    }

    // mEnableAccelStruct =  path_tracer_json["EnableAccel"].asBool();
    // mDivide = path_tracer_json["Subdivide"].asInt();
    mResultPath = path_tracer_json["ResultPath"].asString();
    mRayDisplay = path_tracer_json["EnableRayDisplay"].asBool();
    mMaxDepth = path_tracer_json["Depth"].asInt();
    mOpenResult = path_tracer_json["OpenResultAfterDone"].asBool();
    mSamples = path_tracer_json["Samples"].asInt();
    mEnableIndrectLight = path_tracer_json["EnableIndirectLight"].asBool();
    mDrawLight = path_tracer_json["DrawLight"].asBool();
    mAccelStruct = BuildAccelStruct(path_tracer_json["RayCastAccel"]);

    assert(mAccelStruct != nullptr);
    assert(mMaxDepth > 0);
    assert(mResultPath.size() > 0);
    assert(mSamples > 0);
    // std::cout <<"[debug] mResultPath = " << mResultPath << std::endl;
    // exit(1);
}


void cPathTracer::OutputImage()
{
    cFileUtil::ClearFile(mResultPath);
    FILE *f = cFileUtil::OpenFile(mResultPath, "w");
    fprintf(f, "P3\n%d %d\n%d\n", mWidth, mHeight, 255);
    for (int i=0; i<mHeight * mWidth; i++)
    {
        fprintf(f,"%d %d %d ", static_cast<int>(mScreenPixel[i][0] * 255),
            static_cast<int>(mScreenPixel[i][1] * 255),
            static_cast<int>(mScreenPixel[i][2] * 255));
        // std::cout <<"output " << mScreenPixel[i].transpose() << std::endl;
    }

    cFileUtil::CloseFile(f);
    std::cout <<"[debug] output img to " << mResultPath <<", finished\n";

    // execute command to open it...
#ifdef __APPLE__
    // std::cout <<"try to execute " << cmd << std::endl;
    if(mOpenResult)
        std::string res = cFileUtil::ExecuteCommand("open " + mResultPath);
    // std::cout <<"Res = " << res << std::endl;
#endif
}

void cPathTracer::GetDrawResources(std::vector<tLine> & lines, std::vector<tVertex> & pts, std::vector<tFace> & faces)
{
    lines = mDrawLines;
    pts = mDrawPoints;

    // tRay cur_ray = mScreenRay[mWidth * mHeight / 2 + mWidth / 4];
    // int go = 3;
    // for(int i=0; i<go; i++)
    // {
    //     tLine line;
    //     tVertex st_pt, end_pt;
    //     line.mOri = st_pt.mPos = cur_ray.GetOri();

    //     // set up new ray
    //     tFace * target_face;
    //     tVector inter_pt;
    //     RayCast(cur_ray, inter_pt, &target_face);
    //     tVertex ** vertex_lst = target_face->mVertexPtrList;
    //     tVector normal = ((vertex_lst[1]->mPos - vertex_lst[0]->mPos).cross3(vertex_lst[2]->mPos - vertex_lst[1]->mPos)).normalized();
    //     {
    //         double pdf;
    //         tVector dir = cMathUtil::SampleHemiSphereUniform(normal, pdf);
    //         cur_ray.Init(inter_pt, dir);
    //         std::cout <<"[random ray] normal = " << normal.transpose() <<", dir = " << dir.transpose() << std::endl;
    //         std::cout <<"[new ray] ori = " << cur_ray.GetOri().transpose() <<", dir = " << cur_ray.GetDir().transpose() << std::endl;
    //     }
    //     end_pt.mPos = inter_pt;
    //     line.mDest = inter_pt;
    //     pts.push_back(st_pt), pts.push_back(end_pt);
    //     lines.push_back(line);
    //     std::cout <<"from " << st_pt.mPos.transpose() <<" to " << end_pt.mPos.transpose() << std::endl;
    // }

    // draw light
    if(mDrawLight)
    {
        for(auto & light : mLight)
        {
            if(eLightType::SQUARE == light->GetType())
            {
                std::vector<tFace> s_fs;
                light->GetDrawShape(s_fs);
                for(auto & f : s_fs) faces.push_back(f);
            }
        }
    }


    // test normal
    // double pdf;
    // tVertex v;
    // for(int i=0; i<5000; i++)
    // {
    //     tVector vec = cMathUtil::SampleHemiSphereUniform(tVector(0, -1, 0, 0).normalized(), pdf);
    //     v.mPos = vec;
    //     pts.push_back(v);
    // }
}

void cPathTracer::RayTracing()
{
    std::cout <<"[log] cPathTracer::RayCast begin\n";
    cTimeUtil::Begin();

    std::vector<tFace *> &faces = mSceneMesh->GetFaceList();

    // for(int i=0; i < mWidth * mHeight; i++)
    int inter_num = 0;
#ifdef ENABLE_OPENMP
#pragma omp parallel for schedule(static)
#endif
    for(int x = 0; x< mHeight; x++)
    {
        printf("\rprogress: %.3f%%\n", inter_num++ * 100.0 / (mHeight));
        for(int y = 0; y< mWidth; y++)
        {
            // if(i%10 !=0) continue;
            int i = x * mWidth + y;
            mScreenPixel[i] = tVector::Zero();
            // for(int s=0; s<mSamples; s++)
            {
                mScreenPixel[i] = RayTracePrimaryRay(mScreenRay[i], i);
                // mScreenPixel[i] = RayTracePrimaryRay(mScreenRay[i], i) / mSamples;
            }    
        }
    }
    
    std::cout <<"[log] cPathTracer::RayCast end\n";
    cTimeUtil::End();

    OutputImage();
}

tVector cPathTracer::RayTracePrimaryRay(const tRay & ray_, int ray_id) const
{
    assert(cMathUtil::IsPoint(ray_.GetOri()));
    assert(cMathUtil::IsVector(ray_.GetDir()));
    tRay ray;
    // tVector color = tVector::Zero(); 

    // step2: find intersection
    tRay wi_ray;    // from light source to ref pt
    double pdf, cos_theta_normal_wi;
    tVector brdf_value, ref_normal, ref_pt = tVector::Ones() * std::nan("");
    tFace * target_face = nullptr;
    std::shared_ptr<cBxDF> brdf;
    int mat_id = -1;
    tVector final_color = tVector::Zero(), color = tVector::Zero();
    tMaterial * mat = nullptr;
    std::vector<tVector> direct_light_lst(0), indirect_light_lst(0);
    tVector direct_color = tVector::Zero(), indirect_coeff = tVector::Zero();

    // begin
    for(int s=0; s< this->mSamples; s++)
    {
        ray = ray_;
        ref_pt = tVector::Ones() * std::nan("");
        target_face = nullptr;
        mat_id = -1;
        final_color = tVector::Zero();
        mat = nullptr;
        direct_light_lst.clear(), indirect_light_lst.clear();
        direct_color = indirect_coeff = tVector::Zero();

        for(int bounce = 0; bounce < mMaxDepth; bounce++)
        {
            direct_color = tVector::Zero();
            indirect_coeff = tVector::Zero();

            // 1. raycast, get target face and material
            // target_face = RayCast(ray, ref_pt);
            target_face = mAccelStruct->RayCast(ray, ref_pt);
            // target_face = RayCast(ray, ref_pt, mEnableAccelStruct, mSceneMesh, mAABBLst);
            if(target_face == nullptr)
            {
                final_color = tVector::Zero();
                break;
            }

            mat_id = target_face->mMaterialId;

            // 2. if material is empty
            if(mat_id == -1)
            {
                break;
            }
            mat = mSceneMesh->GetMaterial(mat_id);
            // return mat->diffuse;
            
            // 3. if material is ambient, set up final color, break, do not reflect any more
            if(mat->ambient.norm() > 1e-6)
            {
                final_color = mat->ambient;
                break;
            }

            brdf = mBxDF[mat_id];
            tVertex ** vertex_lst = target_face->mVertexPtrList;
            ref_normal = ((vertex_lst[1]->mPos - vertex_lst[0]->mPos).cross3(vertex_lst[2]->mPos - vertex_lst[1]->mPos)).normalized();


            // 4. calculate direct light
            {
                for(auto & cur_light : mLight)
                {
                    assert(eLightType::SQUARE == cur_light->GetType());
                    // 从光源中采样一个点，
                    tVector Li = tVector(10, 10, 10, 0);
                    // wi_ray: from light source to pt
                    // pdf = r^2 / (A * cos_theta_0), if illegal, pdf = 0
                    cur_light->Sample_Li(ref_pt, wi_ray, &pdf);

                    // intersect with myself?
                    if(pdf < 1e-10) continue;

                    // visible test failed -> no light avaliable, continue
                    // if(false == VisTestForLight(wi_ray.GetOri(), ref_pt, false, mSceneMesh, mAABBLst)) 
                    if(false == mAccelStruct->VisTest(wi_ray.GetOri(), ref_pt))
                        continue;

                    // L_i * cos(theta_i) * cos(theta_0) * A / r^2
                    // = L_i * cos(theta_i) / p(w)
                    assert(cMathUtil::IsVector(wi_ray.GetDir()));
                    assert(cMathUtil::IsNormalized(wi_ray.GetDir()));
                    cos_theta_normal_wi = ref_normal.dot(-wi_ray.GetDir());
                    if(cos_theta_normal_wi < 0) continue;
                    // {
                    //     std::cout <<"ref normal = " << ref_normal.transpose() << std::endl;
                    //     std::cout <<"wi_ray = " << -wi_ray.GetDir().transpose() << std::endl;
                    //     std::cout <<"ref pt = " << ref_pt.transpose() << std::endl;
                    //     std::cout <<"pdf = " << pdf << std::endl;
                    //     exit(1);
                    // }
                    brdf_value = brdf->evaluate(wi_ray.GetDir(), -ray.GetDir(), ref_normal);/* * M_PI * 2;*/
                    direct_color += cMathUtil::WiseProduct(Li * cos_theta_normal_wi / pdf, brdf_value);
                    assert(direct_color.minCoeff() > -1e-6);
                    // if(false == direct_color.minCoeff() > -1e-6)
                    // {
                    //     std::cout <<"direct color negative = " <<  direct_color.transpose() << std::endl;
                    //     std::cout <<"Li = " << Li.transpose() << ", theta = " << cos_theta_normal_wi << ", pdf = " << pdf << ", brdf = " << brdf_value.transpose() << std::endl;
                    //     exit(1);
                    // }
                    // direct_color += cMathUtil::WiseProduct(Li * cos_theta_normal_wi / pdf, brdf_value) / mSamples;
                }
            }

            // calculate indirect light
            // if no indrect light, no out put ray
            if(mEnableIndrectLight == true)
            {
                // 1. decide a random sample ray: wi_dir from ref pt to light src
                tVector wi_dir = cMathUtil::SampleHemiSphereUniform(ref_normal, pdf);
                wi_ray.Init(ref_pt, -wi_dir);   // wi_ray from light src to ref_pt

                brdf_value = brdf->evaluate(wi_ray.GetDir(), -ray.GetDir(), ref_normal);
                indirect_coeff = brdf_value * ref_normal.dot(wi_dir) / pdf;
                // 2. calculate the coeff
                /*
                    coeff = brdf * cos(face_normal * w_i) / pdf
                */
                // change light dir
                ray.Init(ref_pt, wi_dir);
            
                direct_light_lst.push_back(direct_color);
                indirect_light_lst.push_back(indirect_coeff);
                assert(indirect_coeff.minCoeff() > -1e-10);
                // if(indirect_coeff.minCoeff() < -1e-6)
                // {
                //     std::cout << " for ray " << ray_id <<" bounce " << bounce <<", normal = " << ref_normal.transpose()<<std::endl;
                //     std::cout << "indirect light = " << indirect_coeff.transpose() << std::endl;
                //     std::cout <<"brdf = " << brdf_value.transpose() << std::endl;
                //     std::cout <<"cos theta = " << ref_normal.dot(wi_dir) << std::endl;
                //     std::cout <<"ref_normal = " << ref_normal.transpose() << std::endl;
                //     std::cout <<"wi dir = " << wi_dir.transpose() << std::endl;
                //     std::cout <<"pdf = " << pdf << std::endl;
                //     exit(1);
                // }
            }
            else
            {
                ray.Init(tVector::Zero(), tVector::Zero());

                direct_light_lst.push_back(direct_color);
                indirect_light_lst.push_back(indirect_coeff);
                break;
            }
        }
    
        // stack & get color
        for(int i=direct_light_lst.size()-1; i>=0; i--)
        {
            final_color = direct_light_lst[i] + cMathUtil::WiseProduct(indirect_light_lst[i], final_color);

            if(final_color.maxCoeff() > 1)
            {
                final_color /= final_color.maxCoeff();
            }
        }
        color += final_color /mSamples;
    }
    
    if(color.maxCoeff() > 1) color /= color.maxCoeff();
    return color;
}