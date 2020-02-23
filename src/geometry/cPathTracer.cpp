#include "cPathTracer.hpp"
#include <geometry/cBaseMesh.h>
#include <render/camera/cBaseCamera.hpp>
#include <util/cTimeUtil.hpp>
#include <util/cFileUtil.h>
#include <util/cJsonUtil.hpp>
#include <geometry/cLight.hpp>
#include <geometry/cBxDF.hpp>
#include <omp.h>
// #define DEBUG_MODE
#define ENABLE_OPENMP

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

    if(mAccelStructure)
    {
        BuildAccelStructure();
    }

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

void cPathTracer::BuildAccelStructure()
{
    
    // create AABB and divide the space
    if(nullptr == mSceneMesh)
    {
        std::cout <<" scene null failed\n";
        exit(1);
    }

    tVector upper, lower;
    mSceneMesh->GetBound(upper, lower);
    for(int i=0; i<3; i++) upper[i] +=1e-5, lower[i]-=1e-5;
    // std::cout <<"[debug] upper = " << upper.transpose() <<", lower = " << lower.transpose() << std::endl;
    
    // divide the whole space
    tAABB box;
    mAABBLst.clear();
    int id[3] = {0, 0, 0};
    for(id[0]=0; id[0]<mDivide; id[0]++)
        for(id[1]=0; id[1]<mDivide; id[1]++)
            for(id[2]=0; id[2]<mDivide; id[2]++)
    {
        for(int i=0; i<3; i++)
        {
            box.bound[i][0] = lower[i] + (upper-lower)[i] / mDivide * id[i];
            box.bound[i][1] = lower[i] + (upper-lower)[i] / mDivide * (id[i] + 1);
        }
        mAABBLst.push_back(box);
    }
    
    // dispatch all faces to these AABB
    std::vector<tFace *> & faces = mSceneMesh->GetFaceList();
#ifdef ENABLE_OPENMP
#pragma omp parallel for
#endif 
    for(int i=0; i<faces.size(); i++)
    {
        auto & face = faces[i];
        bool find_box = false;
        for(int i=0; i<mAABBLst.size(); i++)
        {
            auto &box = mAABBLst[i];
            // std::cout <<"find " << i << std::endl;
            if(true == box.intersect(face))
            {
                find_box = true;
                // std::cout <<"intersect good =  " << i << std::endl;
#ifdef ENABLE_OPENMP
            #pragma omp critical
#endif
                box.mFaceId.push_back(face->mFaceId);
            }
        }

        // check and verify
        if(false == find_box)
        {
            std::cout <<"[error] find box for face failed" << std::endl;
            for(int i=0; i<NUM_VERTEX_PER_FACE; i++)
                std::cout << face->mVertexPtrList[i]->mPos.transpose() << std::endl;

            std::cout << "all boxes info\n";
            for(auto &box : mAABBLst)
            {
                std::cout <<"-----------\n";
                std::cout <<"box x range = " << box.bound[0].transpose() << std::endl;
                std::cout <<"box y range = " << box.bound[1].transpose() << std::endl;
                std::cout <<"box z range = " << box.bound[2].transpose() << std::endl;
            }
            exit(1);
        }
    }

    // remove all empty boxes
    mAABBLst.erase(std::remove_if(mAABBLst.begin(), mAABBLst.end(), [](const tAABB & box) { return box.mFaceId.size() == 0; }), mAABBLst.end());
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

    mAccelStructure =  path_tracer_json["EnableAccel"].asBool();
    mDivide = path_tracer_json["Subdivide"].asInt();
    mResultPath = path_tracer_json["ResultPath"].asString();
    mRayDisplay = path_tracer_json["EnableRayDisplay"].asBool();
    mMaxDepth = path_tracer_json["Depth"].asInt();
    mOpenResult = path_tracer_json["OpenResultAfterDone"].asBool();
    mSamples = path_tracer_json["Samples"].asInt();
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

void cPathTracer::RayTracing()
{
    std::cout <<"[log] cPathTracer::RayCast begin\n";
    cTimeUtil::Begin();

    std::vector<tFace *> &faces = mSceneMesh->GetFaceList();

    // for(int i=0; i < mWidth * mHeight; i++)
    int inter_num = 0;
    for(int x = 0; x< mHeight; x++)
    {
        printf("\rprogress: %.3f%%", x * 100.0 / (mHeight));
#ifdef ENABLE_OPENMP
#pragma omp parallel for schedule(static)
#endif
        for(int y = 0; y< mWidth; y++)
        {
            // if(i%10 !=0) continue;
            int i = x * mWidth + y;
            mScreenPixel[i] = RayTracePrimaryRay(mScreenRay[i], 0, mMaxDepth, mAccelStructure, mSceneMesh, mAABBLst, mSamples);
            // if(mScreenPixel[i].norm() > 1e-3)
            // {
            //     std::cout <<"get color = " << mScreenPixel[i].transpose() << " ";
            // } 
// #ifdef DEBUG_MODE
//             if(inter == true)
//             {
//                 if(i % 100 == 0 && mRayDisplay == true)
//                 {
//                     tLine line;
//                     tVertex vertex;
//                     // std::cout << pos.transpose() << std::endl;
//                     line.mOri = ray.GetOri();
//                     line.mDest = ray.GetOri() + 10e4 * ray.GetDir();
//                     line.mColor = tVector(0, 1, 0, 1);
//                     vertex.mPos = pos;
//                     vertex.mColor = tVector(0.5, 0.5, 0.5, 1);
// #pragma omp critical
//                     mDrawLines.push_back(line);
//                     mDrawPoints.push_back(vertex);
//                 }
// #pragma omp critical
//                 inter_num++;
//             }
// #endif
        }
    }
    
    // std::cout << "[debug] total rays = " << mHeight * mWidth << std::endl;
    // printf("\n");
    // std::cout <<"intersections = " << inter_num << std::endl;
   

    std::cout <<"[log] cPathTracer::RayCast end\n";
    cTimeUtil::End();

    OutputImage();
}

tVector cPathTracer::RayTracePrimaryRay(tRay & ray, int depth, const int mMaxDepth,const bool mAccelStructure,const std::shared_ptr<cObjMesh> mSceneMesh,const std::vector<tAABB> & mAABBLst, const int samples)
{
    tVector color = tVector::Zero(); 

    // step1: check depth
    if(depth > mMaxDepth) return color;

    // step2: find intersection
    tVector pt = tVector::Ones() * std::nan("");
    tFace * target_face = nullptr;

    // step3: if there is an intersection, begin to sample
    if(RayCast(ray, pt, &target_face, mAccelStructure, mSceneMesh, mAABBLst) == true)
    {
        int mat_id = target_face->mMaterialId;
       
        if(mat_id == -1)
        {
            // 没有材质: 白色
            return tVector(1, 1, 1, 0);
        }
        tMaterial * mat = mSceneMesh->GetMaterial(mat_id);
        // if(mat->ambient.norm() > 1e-6)
        // {

        //     // for(int i=0; i<3; i++)
        //         // std::cout << target_face->mVertexPtrList[i]->mPos.transpose() << std::endl;
        //     // std::cout << "light! = " << pt.transpose() << std::endl;
        // }
        std::shared_ptr<cBxDF> brdf = mBxDF[mat_id];
        tVertex ** vertex_lst = target_face->mVertexPtrList;
        tVector normal = ((vertex_lst[1]->mPos - vertex_lst[0]->mPos).cross3(vertex_lst[2]->mPos - vertex_lst[1]->mPos)).normalized();

        // std::cout <<"----------------"
        std::vector<double> pdfs;
        std::vector<tVector> brdfs;
        // direct lighting
        {
            // iter over all lights
            for(int i=0; i<mSamples; i++)
            {
                tRay light_ray;
            double pdf;
            for(auto & cur_light : mLight)
            {
                assert(eLightType::SQUARE == cur_light->GetType());
                // 从光源中采样一个点，
                tVector Li = tVector(10, 10, 10, 0);
                // light_ray: from light source to pt
                // pdf = r^2 / (A * cos_theta_0), if illegal, pdf = 0
                cur_light->Sample_Li(pt, light_ray, &pdf);
                pdfs.push_back(pdf);
                if(pdf < 1e-10) // pdf == 0: illegal
                {
                    // std::cout <<"pdf == 0, continue\n";
                    continue;
                }
                // visible test failed -> no light avaliable, continue
                if(false == VisTestForLight(light_ray.GetOri(), pt, mAccelStructure, mSceneMesh, mAABBLst))
                {

                    // if(std::fabs(pt[1] - 0.5) < 1e-10)
                    // {
                    //     std::cout <<"debug cur pt = " << pt.transpose() <<" invisible ! its impossible\n";
                    //     exit(1);
                    // }
                    continue;
                }
                // L_i * cos(theta_i) * cos(theta_0) * A / r^2
                // = L_i * cos(theta_i) / p(w)
                assert(cMathUtil::IsVector(light_ray.GetDir()));
                if(false == cMathUtil::IsNormalized(light_ray.GetDir()))
                {
                    std::cout <<"ori = " << light_ray.GetOri().transpose() << std::endl;
                    std::cout <<"dir = " << light_ray.GetDir().transpose() << std::endl;
                    std::cout <<"pdf = " << pdf << std::endl;
                    exit(1);
                }
                double cos_theta_i = normal.dot(-light_ray.GetDir());
                tVector brdf = mBxDF[mat_id]->evaluate(light_ray.GetDir(), -ray.GetDir(), normal);/* * M_PI * 2;*/
                brdfs.push_back(brdf);
                color += cMathUtil::WiseProduct(Li * cos_theta_i / pdf, brdf) / mSamples;
                }
            }
            // if(pt[1] - 0.5 < 1e-10 && color.norm() < 0.2)
            // {
            //     std::cout <<"pt = " << pt.transpose() <<" color = " << color.transpose() << std::endl;
            //     std::cout <<"num brdf = " << brdfs.size() <<", pdf num = " << pdfs.size() << std::endl;
            //     for(int i=0; i<mSamples; i++)
            //     {
            //         std::cout << i <<" brdf = " << brdfs[i].transpose() <<", pdf = " << pdfs[i] << std::endl;
            //     }
            //     exit(1);
            // }
        }

        // indirect light
        // if(mat->ambient.norm() > 1e-5)
        // {
        //     // light source: 返回光源色
        //     color = mat->ambient;
        // }
        // else
        // {
        //     // 是其他的, 开始半球采样
        //     for(int i=0; i<mSamples; i++)
        //     {
        //         double pdf;
        //         tVector dir = cMathUtil::SampleHemiSphereUniform(normal, pdf);
        //         // dir = (tVector(55, 22.5, 22.5, 0) - pt).normalized();
        //         ray.Init(pt, dir);

        //         tVector brdf = mat->diffuse / M_PI;
        //         tVector Li = RayTracePrimaryRay(ray, depth + 1, mMaxDepth, mAccelStructure, mSceneMesh, mAABBLst, mSamples);
        //         double cos = normal.dot(dir);
        //         // color += BRDF * Li * cos / pdf
        //         color += tVector(brdf.array() * Li.array() * cos / pdf);
        //         // if(Li.norm() > 1e-10)
        //         // {
        //         //     std::cout <<"get li = " << Li.transpose() << std::endl;
        //         //     std::cout <<"cur color = " << color.transpose() << std::endl;
        //         // }
        //     }
        //     // std::cout <<"sample = " <<  mSamples<<std::endl;
        //     color /= mSamples;
        // }
    }
    // std::cout <<"return color = " << color.transpose() << std::endl;
    // if(depth == 0 && color.norm() > 1e-5)
    // {
    //     std::cout <<"color = " << color.transpose() << std::endl;
    // }
    // std::cout << "color = " << color.transpose() << std::endl;
    if(color.maxCoeff() > 1)
    {
        color /= color.maxCoeff();
        // color.normalize();
    }
    // for(int i=0; i<3; i++)
    // {
    //     cMathUtil::clamp(color[i], 1.0, 0.0);
    // } 
    return color;
}

/*
    ori: the origin of the ray
    dir: the direction or the ray
    pos: interseciton points
    face: pointer to the pointer of a intersection face
    return bool: intersection or not?
*/
bool cPathTracer::RayCast(const tRay & ray, tVector & pt, tFace ** target_face,const bool mAccelStructure,const std::shared_ptr<cObjMesh> mSceneMesh,const std::vector<tAABB> & mAABBLst)
{
    pt = tVector::Ones() * std::nan("");
    *target_face = nullptr;

    bool intersect = false;
    double dist = std::numeric_limits<double>::max();
    std::vector<tFace *> & full_faces = mSceneMesh->GetFaceList();
    if(mAccelStructure == false)
    {
        // std::cout <<" ray ori = " << ray.GetOri().transpose() << std::endl;
        // std::cout <<"ray dir = " << ray.GetDir().transpose() << std::endl;
        for(auto &x : full_faces)
        {
            tVector p = cMathUtil::RayCast(ray.GetOri(), ray.GetDir(), x->mVertexPtrList[0]->mPos,
                x->mVertexPtrList[1]->mPos,
                x->mVertexPtrList[2]->mPos);
            if(p.hasNaN() == false && (p - ray.GetOri()).norm() < dist && (p-ray.GetOri()).norm() > 1e-6)
            {
                pt = p;
                dist = (p - ray.GetOri()).norm();
                *target_face = x;
                intersect = true;
            }
        }
    }
    else
    {
        // ray-aabb intersect -> ray face intersect
        for(auto & box : mAABBLst)
        {
            if(box.intersect(ray) == false) continue;
            for(auto & id : box.mFaceId)
            {
                auto & x =  full_faces[id];
                tVector p = cMathUtil::RayCast(ray.GetOri(), ray.GetDir(), x->mVertexPtrList[0]->mPos,
                    x->mVertexPtrList[1]->mPos,
                    x->mVertexPtrList[2]->mPos);
                if(p.hasNaN() == false && (p - ray.GetOri()).norm() < dist && (p-ray.GetOri()).norm() > 1e-6)
                {
                    pt = p;
                    dist = (p - ray.GetOri()).norm();
                    *target_face = x;
                    intersect = true;
                }
            }
        }
    }

    return intersect;
}

// the light is from p1 to p2
bool cPathTracer::VisTestForLight(const tVector & p1, const tVector & p2,const bool mAccelStructure,const std::shared_ptr<cObjMesh> mSceneMesh,const std::vector<tAABB> & mAABBLst)
{
    double dist = (p1 - p2).norm();
    double cur_min_dist = dist;
    tVector light_dir = (p2 - p1).normalized();
    std::vector<tFace *> & full_faces = mSceneMesh->GetFaceList();
    if(mAccelStructure == false)
    {
        // std::cout <<" ray ori = " << ray.GetOri().transpose() << std::endl;
        // std::cout <<"ray dir = " << ray.GetDir().transpose() << std::endl;
        for(auto &x : full_faces)
        {
            double p = cMathUtil::RayCastT(p1, light_dir, x->mVertexPtrList[0]->mPos,
                x->mVertexPtrList[1]->mPos,
                x->mVertexPtrList[2]->mPos);
            assert(std::isnan(p) || p > -1e-6); // if p <=0, illegal

            // p > 1e-6: in case of the intersection is with light it self
            if(std::isnan(p) == false && p < cur_min_dist - 1e-6 && p > 1e-6)
            {
                cur_min_dist = p;
                // 如果是0.5附近却不可见，这就不对
                // if(std::fabs(p2[1] - 0.5) < 1e-10)
                // {
                //     std::cout <<"[visible test] pt = " << p2.transpose() << std::endl;
                //     std::cout <<"dist = " << dist <<" cur dist = " << cur_min_dist << std::endl;
                //     exit(1);
                // }
                // it's invisible if here is an intersect
                if(cur_min_dist < dist) return false;
            }
        }
    }
    else
    {
        // ray-aabb intersect -> ray face intersect
        tLine cur_line;
        cur_line.mOri = p1; cur_line.mDest = p2;
        for(auto & box : mAABBLst)
        {
            if(box.intersect(cur_line) == false) continue;
            for(auto & id : box.mFaceId)
            {
                auto & x =  full_faces[id];
                double p = cMathUtil::RayCastT(p1, light_dir, x->mVertexPtrList[0]->mPos,
                    x->mVertexPtrList[1]->mPos,
                    x->mVertexPtrList[2]->mPos);
                assert(std::isnan(p) || p > 0);
                if(std::isnan(p) == false && p < cur_min_dist - 1e-6 && p > 1e-6)
                {
                    cur_min_dist = p;

                    // it's invisible if here is an intersect
                    if(cur_min_dist < dist)
                    {
                        // 如果是0.5附近却不可见，这就不对
                        // if(std::fabs(p2[1] - 0.5) < 1e-10)
                        // {
                        //     std::cout <<"[visible test] pt = " << p2.transpose() << std::endl;
                        //     std::cout <<"dist = " << dist <<" cur dist = " << cur_min_dist << std::endl;
                        //     exit(1);
                        // }
                        return false;
                        
                    } 
                }
                // if(p.hasNaN() == false && (p - ray.GetOri()).norm() < dist && (p-ray.GetOri()).norm() > 1e-6)
                // {
                //     pt = p;
                //     dist = (p - ray.GetOri()).norm();
                //     *target_face = x;
                //     intersect = true;
                // }
            }
        }
    }

    return true;
}