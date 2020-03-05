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
    if(mEnablePathTracing == true) RayTracing();
    else std::cout <<"cPathTracer::Process: path tracing is disabled\n"; 

    // tVector vec = tVector::Random();
    // std::cout << vec.transpose() << std::endl;
    // std::cout << cMathUtil::SkewMat(vec);
    // exit(1);
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
    std::cout <<"[log] cPathTracer::InitAccelStruct begin\n";
    cTimeUtil::Begin();
    mAccelStruct->Init(std::static_pointer_cast<cBaseMesh>(mSceneMesh));
    cTimeUtil::End();
    std::cout <<"[log] cPathTracer::InitAccelStruct end\n";
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
    // exit(1);
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
        tVector test = tVector(0 , 0, 0, 1);
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
        // mat3(0, 0) = std::tan(cMathUtil::Radians(mFov) / 2) * mNear;
        mat3(0, 0) = mWidth * 1.0 / mHeight * std::tan(cMathUtil::Radians(mFov) / 2) * mNear;
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
    mEnableBarycentricNormal = path_tracer_json["EnableBarycentricNormal"].asBool();
    mAccelStruct = BuildAccelStruct(path_tracer_json["RayCastAccel"]);
    mEnablePathTracing = path_tracer_json["EnablePathTracing"].asBool();
    // std::cout << mEnablePathTracing << std::endl;
    // exit(1);
    {
        Json::Value DrawRegion_json = path_tracer_json["DrawRegion"];
        assert(DrawRegion_json.isNull()==false && DrawRegion_json.size() == 4);
        mDrawRegion.stX = DrawRegion_json[0].asInt();
        mDrawRegion.stY = DrawRegion_json[1].asInt();
        mDrawRegion.edX = DrawRegion_json[2].asInt();
        mDrawRegion.edY = DrawRegion_json[3].asInt();
        printf("[debug] draw region = (%d, %d) to (%d, %d)\n", mDrawRegion.stX, mDrawRegion.stY, mDrawRegion.edX, mDrawRegion.edY);
    }
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
std::vector<tVector> reflection_pts;
void cPathTracer::GetDrawResources(std::vector<tLine> & lines, std::vector<tVertex> & pts, std::vector<tFace> & faces)
{
    lines = mDrawLines;
    pts = mDrawPoints;


    if(mRayDisplay)
    {
        const int gap = 100;
        tVertex v;
        for(int id = 0; id < mWidth * mHeight ; id+=gap)
        {
            tRay cur_ray = mScreenRay[id];
            tLine line;
            
            // tVertex end_pt;
            // line.mOri = st_pt.mPos = cur_ray.GetOri();
            // set up new ray
            // tFace * target_face;
            // tVector inter_pt;
            // RayCast(cur_ray, inter_pt, &target_face);
            // tVertex ** vertex_lst = target_face->mVertexPtrList;
            // tVector normal = ((vertex_lst[1]->mPos - vertex_lst[0]->mPos).cross3(vertex_lst[2]->mPos - vertex_lst[1]->mPos)).normalized();
            // {
            //     double pdf;
            //     tVector dir = cMathUtil::SampleHemiSphereUniform(normal, pdf);
            //     cur_ray.Init(inter_pt, dir);
            //     std::cout <<"[random ray] normal = " << normal.transpose() <<", dir = " << dir.transpose() << std::endl;
            //     std::cout <<"[new ray] ori = " << cur_ray.GetOri().transpose() <<", dir = " << cur_ray.GetDir().transpose() << std::endl;
            // }
            // end_pt.mPos = ;
            mAccelStruct->RayCast(cur_ray, v.mPos);
            line.mOri = cur_ray.GetOri();
            line.mDest = line.mOri + cur_ray.GetDir() * 100;
            // pts.push_back(st_pt), pts.push_back(end_pt);
            lines.push_back(line);
            pts.push_back(v);
            // std::cout <<"from " << st_pt.mPos.transpose() <<" to " << end_pt.mPos.transpose() << std::endl;
        }
    }
    // std::cout <<"lines num = " << lines.size() << std::endl;

    // draw light
    if(mDrawLight)
    {
        for(auto & light : mLight)
        {
            std::vector<tFace> s_fs;
            light->GetDrawShape(s_fs);
            for(auto & f : s_fs) faces.push_back(f);
        }
    }

    // test normal
    // double pdf;
    // tVertex v;
    // tVector sum = tVector::Zero();
    // for(int i=0; i<500; i++)
    // {
    //     tVector vec = cMathUtil::SampleSphereUniform(tVector(0, -1, 0, 0).normalized(), pdf);
    //     v.mPos = vec;
    //     pts.push_back(v);
    //     sum += vec;
    // }
    // std::cout << sum.transpose() << std::endl;

    // test fraction
    // {
    //     // std::cout <<  << std::endl;
    //     std::shared_ptr<cBxDF> bxdf = nullptr;
    //     for(int i=0; i< mBxDF.size(); i++)
    //     {
    //         if(mBxDF[i]->GetType() == eBxDFType::BSDF)
    //         {
    //             bxdf = mBxDF[i];
    //             std::cout << i <<" is bsdf\n"<< std::endl;
    //             break;
    //         }
            
    //     } 
    //     // tVector in_dir = -tVector(drand48(), drand48(), 1, 0).normalized();
    //     double pdf;
    //     tVector normal = tVector(0, 1, 0, 0).normalized();
    //     // tVector normal = cMathUtil::SampleHemiSphereUniform(tVector(0, 1, 0, 0), pdf);
    //     tLine income, refract_out, normal_line;
    //     // 0.0951017 -0.746427  0.658637
    //     tVector in_dir = tVector(-0.022489,  0.537434, -0.843006, 0).normalized();
    //     // tVector in_dir = cMathUtil::SampleHemiSphereUniform(-normal, pdf).normalized();
    //     income.mOri = tVector(0, 0, 0, 1) - 2 * in_dir;
    //     income.mDest = tVector(0, 0, 0, 1);
    //     income.mColor = tVector(1, 1, 1, 1);
    //     normal_line.mOri = tVector(0, 0, 0, 1);
    //     normal_line.mDest = normal * 2;
    //     normal_line.mColor = tVector(0.5, 0.5, 0.5, 1);
    //     refract_out.mOri = tVector(0, 0, 0, 1);
    //     refract_out.mDest = cGeoUtil::Refract(normal, in_dir, 1.0 / 1.5) * 2;
    //     // assert(refract_out.mDest.hasNaN() == false);
    //     refract_out.mColor = tVector(0, 0, 0, 1);
    //     lines.push_back(income);
    //     lines.push_back(refract_out);
    //     lines.push_back(normal_line);
    //     for(int i=0; i<10000; i++)
    //     {
    //         // tVector in_dir = -cMathUtil::SampleHemiSphereUniform(normal, pdf);
    //         tVector wo_dir = cMathUtil::SampleSphereUniform(pdf).normalized();
    //         tVector bxdf_value = bxdf->evaluate(in_dir, wo_dir, normal);
    //         normal_line.mOri = tVector(0, 0, 0, 1);
    //         normal_line.mDest = wo_dir;
    //         normal_line.mColor = tVector::Ones() * bxdf_value.norm();
    //         if(bxdf_value.hasNaN() == true) normal_line.mColor = tVector(0.5, 0.1, 0.9, 1);
    //         lines.push_back(normal_line);
    //         // std::cout <<"bxdf_value = " << bxdf_value.transpose() << std::endl;
    //     }
        
    //     std::cout <<"白色 入射\n";
    //     std::cout <<"灰色 法向\n";
    //     std::cout <<"黑色 折射\n";
    // }

    tLine line;
    for(int i=0; i<reflection_pts.size(); i++)
    {
        tVertex v;
        v.mPos = reflection_pts[i];
        v.mColor = tVector::Ones() * 0.1 * i + tVector::Ones() * 0.5;
        pts.push_back(v);
        if(i == reflection_pts.size() -1) continue;
        line.mOri = reflection_pts[i];
        line.mDest = reflection_pts[i+1];
        line.mColor = tVector(1, 1, 1, 0);
        lines.push_back(line);
    }

    // test samplecone
    // const tVector medial = tVector(1, 1, 1, 0).normalized();
    // const double max_theta = 30.0 / 180 * M_PI;
    // double pdf;
    // tVertex v;
    // const int times = 1e5;
    // tVector sum = tVector::Zero();
    // for(int i=0; i<times; i++)
    // {
    //     v.mPos = cMathUtil::SampleCone(medial, max_theta, pdf);
    //     pts.push_back(v);
    //     sum += v.mPos / times;
    // }
    // std::cout <<" avg = " << sum.normalized().transpose() <<":" << medial.transpose() << std::endl;
    
    // barycentric coordinates test code
    // for(int i=0; i<10000; i++)
    // {
    //     tVertex vertex;
    //     tVector v[3];
    //     tVector weight = tVector::Zero();
    //     tVector res = tVector::Zero();
    //     tFace f;
    //     for(int j=0; j<3; j++)
    //     {
    //         v[j] = tVector::Random();
    //         weight[j] = drand48();
    //         v[j][3] = 1;
    //         res += v[j] * weight[j];
    //         f.mVertexPtrList[j] = new tVertex();
    //         f.mVertexPtrList[j]->mPos = v[j];
    //     }
    //     double scale = weight.sum();
    //     // std::cout <<"debug scale = " << scale << std::endl;
    //     weight /= scale;
    //     res /= scale;
    //     res[3] = 1;
    //     // std::cout <<"[debug] begin calc barycentric coords\n";
    //     tVector bary_coor = cGeoUtil::CalcBarycentricCoordinate(res, v[0], v[1], v[2]);
    //     // std::cout <<"[debug] end calc barycentric coords\n";
    //     // std::cout <<"weight ideal = " << weight.transpose() << std::endl;
    //     // std::cout <<"weight calc = " << bary_coor.transpose() << std::endl;
    //     assert(cMathUtil::IsSame(weight, bary_coor));
    //     vertex.mPos = res;
    //     pts.push_back(vertex);
    //     faces.push_back(f);
    // }
    
    // std::cout <<"final lines num = " << lines.size() << std::endl;
    // exit(1);
}

void cPathTracer::RayTracing()
{
    std::cout <<"[log] cPathTracer::RayCast begin\n";
    cTimeUtil::Begin();

    std::vector<tFace *> &faces = mSceneMesh->GetFaceList();

    // for(int i=0; i < mWidth * mHeight; i++)
    int inter_num = 0;
#ifdef ENABLE_OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
    for(int x = 0; x< mHeight; x++)
    {
        printf("\rprogress: %.3f%%\n", inter_num++ * 100.0 / (mHeight));
        if(x<mDrawRegion.stX || x>mDrawRegion.edX) continue;
        for(int y = 0; y< mWidth; y++)
        {
            if(y<mDrawRegion.stY || y>mDrawRegion.edY) continue;
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

    int height = ray_id / mWidth, width = ray_id % mWidth;
    assert(cMathUtil::IsPoint(ray_.GetOri()));
    assert(cMathUtil::IsVector(ray_.GetDir()));
    tRay ray;
    // tVector color = tVector::Zero(); 

    // step2: find intersection
    tRay wi_ray;    // from light source to ref pt
    double pdf, cos_theta_normal_wi;
    tVector brdf_value, ref_normal, ref_pt = tVector::Ones() * std::nan("");
    tFace * target_face = nullptr;
    std::shared_ptr<cBxDF> bxdf;
    int mat_id = -1;
    tVector final_color = tVector::Zero(), color = tVector::Zero();
    tMaterial * mat = nullptr;
    std::vector<tVector> direct_light_lst(0), indirect_light_lst(0);
    tVector direct_color = tVector::Zero(), indirect_coeff = tVector::Zero();
    // bool first_cast_bsdf = false;
    // tVector first_normal = tVector::Zero();
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
            if(target_face == nullptr || (mat_id = target_face->mMaterialId) == -1)
            {
                final_color = tVector::Zero();
                break;
            }

            
            mat = mSceneMesh->GetMaterial(mat_id);
            // return mat->diffuse;
            
            // 3. if material is ambient, set up final color, break, do not reflect any more
            if(mat->ambient.norm() > 1e-6)
            {
                // double r = (ref_pt - ray.GetOri()).norm();
                // final_color = mat->ambient / pow(r,2);
                final_color = mat->ambient;
                break;
            }

            bxdf = mBxDF[mat_id];
            tVertex ** vertex_lst = target_face->mVertexPtrList;
            if(true == mEnableBarycentricNormal)
            {
                tVector bary_coords = cGeoUtil::CalcBarycentricCoordinate(ref_pt, vertex_lst[0]->mPos, vertex_lst[1]->mPos, vertex_lst[2]->mPos);
                ref_normal = tVector::Zero();
                for(int i=0; i<3; i++ ) ref_normal += bary_coords[i] * vertex_lst[i]->mNormal;
                ref_normal.normalize();
            }
            else
            {
                ref_normal = cGeoUtil::CalcNormalFrom3Pts(vertex_lst[0]->mPos, vertex_lst[1]->mPos, vertex_lst[2]->mPos);
            }
            
            // if(0 == bounce) first_normal = ref_normal;

            // if(ray_id == 156836)
            // {
            //     std::cout <<"bounce " << bounce <<" material = " << bxdf->GetType() << std::endl;
            //     if(reflection_pts.size() < mMaxDepth)
            //         reflection_pts.push_back(ref_pt);
            // }
            // if(height == 400 && width == 274)
            //     if(reflection_pts.size() < mMaxDepth)
            //         reflection_pts.push_back(ref_pt);
            // 4. calculate direct light
            direct_color = bxdf->Sample_Li(mLight, mAccelStruct.get(), ref_normal, ref_pt, ray);
            direct_light_lst.push_back(direct_color);

            // calculate indirect light
            // if no indrect light, no output ray
            if(mEnableIndrectLight == true)
            {
                // input: wo
                // output: wi, indirect light coef 
                // 1. decide a random sample ray: wi_dir from ref pt to light src
                tVector wi_dir_oppo; // the definition of wi_dir is a incoming light ref_pt -> outside
                indirect_coeff = bxdf->Sample_f(ref_normal,  -ray.GetDir(), wi_dir_oppo);
                assert(indirect_coeff.minCoeff() > -1e-10);

                // change light dir
                ray.Init(ref_pt, wi_dir_oppo);
                indirect_light_lst.push_back(indirect_coeff);
            }
            else
            {
                indirect_light_lst.push_back(indirect_coeff);
                break;
            }
        }
    
        // stack & get color
        assert(direct_light_lst.size() == indirect_light_lst.size());
        
        for(int i=direct_light_lst.size()-1; i>=0; i--)
        {
            final_color = direct_light_lst[i] + cMathUtil::WiseProduct(indirect_light_lst[i], final_color);

            if(final_color.maxCoeff() > 1)
            {
                final_color /= final_color.maxCoeff();
            }
        }
        // if(height == 400 && width == 274)
        // std::cout <<"final color " << s <<" " << final_color.transpose() << std::endl;
        color += final_color /mSamples;
    }
    // if(first_cast_bsdf && std::fabs(first_normal[1] -1) < 1e-6 && ray_id == 156836)
    // {
    //     // color *= 10;
    //     std::cout <<"ray id " << ray_id <<" " << ray_id / mWidth <<" " << ray_id % mWidth << " color = " << color.transpose() << std::endl;
    //     // color = tVector(0.2, 0.4, 0.5, 1);
    //     // color *=2;
    // }
    if(color.maxCoeff() > 1) color /= color.maxCoeff();
    return color;
}