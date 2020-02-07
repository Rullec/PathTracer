#include "cPathTracer.hpp"
#include <geometry/cBaseMesh.h>
#include <render/camera/cBaseCamera.hpp>
#include <util/cTimeUtil.hpp>
#include <util/cFileUtil.h>
// #include <omp.h>

cPathTracer::cPathTracer(const std::string &conf)
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

    mSceneMesh = scene_mesh; 
    mCamera = camera;

    // std::cout <<"[debug] cPathTracer::Init begin\n";
    mCamera->GetCameraScene(mWidth, mHeight, mFov, mNear);
    mCameraPos = mCamera->GetCameraPos();
    // std::cout <<"[debug] cPathTracer::Init width, height = " << mWidth <<" " << mHeight << ", fov = " << mFov << ", near = " << mNear << std::endl;
    mScreen = new tVector[mWidth * mHeight];
    mScreenRay = new tRay[mWidth * mHeight];
}

void cPathTracer::Update(std::vector<tLine> & rays, std::vector<tVertex>& pts)
{
    if(mCamera == nullptr || mSceneMesh == nullptr)
    {
        std::cout <<"[error] cPathTracer::Process failed\n" << std::endl;
        exit(1);
    }

    UpdatePrimaryRay();
    RayCastPrimaryRay(rays, pts);
    std::cout <<"ray cast pts = " << pts.size() << std::endl;
}

void cPathTracer::UpdatePrimaryRay()
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
        tMatrix mat1 = tMatrix::Identity();
        mat1(0, 0) = 1.0 / mWidth;
        mat1(0, 3) = 0.5 / mWidth;
        mat1(1, 1) = 1.0 / mHeight;
        mat1(1, 3) = 0.5 / mHeight;
        tMatrix mat2 = tMatrix::Identity();
        mat2(0, 0) = 2;
        mat2(0, 3) = -1;
        mat2(1, 1) = -2;
        mat2(1, 3) = 1;
        // pos = mat2 * pos;
        tMatrix mat3 = tMatrix::Identity();
        mat3(0, 0) = mWidth/ mHeight * std::tan(cMathUtil::Radians(mFov));
        mat3(1, 1) = std::tan(cMathUtil::Radians(mFov));
        mat3(2, 2) = 0, mat3(2, 3) = -mNear;
        // pos = mat3 * pos;
        tMatrix mat4 = mCamera->GetViewMat().inverse();
        mat = mat4 * mat3 * mat2 * mat1;
    }
   tVector camera_pos = mCamera->GetCameraPos();
// #pragma omp parallel for
    for(int y = 0; y < mHeight; y ++ )
    {
        for(int x = 0 ; x < mWidth; x++)
        {
            // [row, col]
            int access_id = y * mWidth + x;
            tVector pos = mat * tVector(x, y, 1, 1);
            mScreenRay[access_id].mOri = camera_pos;
            mScreenRay[access_id].mDir = pos - camera_pos;
        }
    }
    // cTimeUtil::End();
}


void cPathTracer::ParseConfig(const std::string & conf)
{
    std::cout <<"[debug] cPathTracer::ParseConfig " << conf << std::endl;
}


void cPathTracer::GetRayLines(std::vector<tLine> &lines)
{
    // line.resize(mHeight * mWidth);
    lines.clear();
    for(int i=0; i< mHeight * mWidth; i++)
    {
        // if(250 <= i / mWidth&& i / mWidth <= 262 &&250 <= i%mWidth && i%mWidth <=262)
        {
            tLine line;
            line.mOri = mScreenRay[i].mOri;
            line.mDest = mScreenRay[i].mDir + mScreenRay[i].mOri;
            line.mColor = tVector(0.2, 0.3, 0.4, 0.5);
            lines.push_back(line);

        }
        // tRay & ray = mScreenRay[i];
        // line[i].mOri = ray.mOri;
        // line[i].mDest = ray.mDir * 100 + ray.mOri;
        // line[i].mColor = tVector(0.2, 0.3, 0.4, 0.5);
    }
    // std::cout <<"[debug] cPathTracer::GetRayLines " << line.size() << std::endl;
}
#include <omp.h>
void cPathTracer::RayCastPrimaryRay(std::vector<tLine> & lines, std::vector<tVertex> & pts)
{
    cTimeUtil::Begin();
    // collect all rays
    // tRay ray = mScreenRay[mWidth * mHeight / 2 + mWidth / 2];
    // ray.mOri = tVector(0 ,0, 0, 1);
    // ray.mDir = tVector::Random();

    std::vector<tFace *> &faces = mSceneMesh->GetFaceList();
    std::cout <<"[debug] RayCast face num = " << faces.size() << std::endl;
    const std::string log = "raycast.log";
    cFileUtil::ClearFile(log);
    FILE * fout = cFileUtil::OpenFile(log, "w");
    tLine line;
    pts.clear();
    tVertex vertex;
    #pragma omp parallel for
    for(int i=0; i < mWidth * mHeight; i++)
    {
        // if(i%10 !=0) continue;
        printf("\r progress: %.3f%", i * 100.0 / (mWidth * mHeight));
        tRay & ray = mScreenRay[i];
        tVector pos = tVector::Ones() * std::nan("");
        RayCastSingleRay(ray, pos);
        if(pos.hasNaN() == false)
        {
            // std::cout << pos.transpose() << std::endl;
            line.mOri = ray.mOri;
            line.mDest = ray.mOri + 10e4 * ray.mDir;
            lines.push_back(line);
            vertex.mPos = pos;
            vertex.mColor = tVector(0.5, 0.5, 0.5, 1);
            pts.push_back(vertex);
        }
    }
    // {
    //     // 范围测试
    //     tVector upper, lower;
    //     mSceneMesh->GetBound(upper, lower);
    //     std::cout <<"mesh range " << lower.transpose() <<" to " << upper.transpose() << std::endl;
    //     for(auto & pt : pts)
    //     {
    //         auto & pos = pt.mPos;
    //         bool exceed = false;
    //         for(int i=0; i<3; i++)
    //         {
    //             if(pos[i] > upper[i] + 1e-5 || pos[i] < lower[i] - 1e-5)
    //             exceed = true;
    //         }
    //         if(exceed == true)
    //         {
    //             std::cout <<"[error] illegal inter " << pos.transpose() << std::endl;
    //         }
    //     }
    // }
    // do ray cast for this ray
    // 把ray cast结果绘制出来: 要求光线line和交点全部得到
    cTimeUtil::End();
}

void cPathTracer::RayCastSingleRay(const tRay & ray, tVector & pt)
{
    pt = tVector::Ones() * std::nan("");
    std::vector<tFace *> & faces = mSceneMesh->GetFaceList();
    double dist = std::numeric_limits<double>::max();
    for(auto &x : faces)
    {
        tVector p = cMathUtil::RayCast(ray.mOri, ray.mDir, x->mVertexPtrList[0]->mPos,
            x->mVertexPtrList[1]->mPos,
            x->mVertexPtrList[2]->mPos);
        if(p.hasNaN() == false && (p - ray.mOri).norm() < dist)
        {
            pt = p;
            dist = (p - ray.mOri).norm();
        }
    }
}