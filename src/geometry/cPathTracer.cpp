#include "cPathTracer.hpp"
#include <geometry/cBaseMesh.h>
#include <render/camera/cBaseCamera.hpp>
#include <util/cTimeUtil.hpp>
#include <util/cFileUtil.h>
#include <util/cJsonUtil.hpp>
#include <omp.h>

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

    if(mAccelStructure)
    {
        BuildAccelStructure();
    }
}

void RayToLine(const tRay & ray, tLine & line)
{
    line.mOri = ray.GetOri();
    line.mDest = ray.GetDir() * 1e4 + line.mOri;
}

void cPathTracer::Update(std::vector<tLine> & rays, std::vector<tVertex>& pts)
{
    if(mCamera == nullptr || mSceneMesh == nullptr)
    {
        std::cout <<"[error] cPathTracer::Process failed\n" << std::endl;
        exit(1);
    }

//     rays = glo_lines;
// {
// //     -0.5 -0.5  0.5    1
// //  
// //
//     tVertex v[3];
//     v[0].mPos = tVector(-0.5, -0.5,  0.5,    1);
//     v[1].mPos = tVector(0.5, -0.5,  0.5,    1);
//     v[2].mPos = tVector( 0.5, 0.5, 0.5,   1);
//     tFace * face = new tFace();
//     face->mVertexPtrList[0] = v;
//     face->mVertexPtrList[1] = v + 1;
//     face->mVertexPtrList[2] = v+2;

//     tAABB box;
//     box.bound[0] = Eigen::Vector2d(0, 0.5001);
//     box.bound[1] = Eigen::Vector2d(-0.5001, 0);
//     box.bound[2] = Eigen::Vector2d(0.25, 0.5001);

//     if(box.intersect(face) == false)
//     {
//         std::cout << " error\n";
//         exit(1);
//     }
// }
    // 测试
    UpdatePrimaryRay();

    std::cout <<"[debug] begin ray cast primary rays\n";
    RayCastPrimaryRay(rays, pts);

    // rays.resize(mWidth * mHeight);
    // for(int i=0; i< mWidth * mHeight; i++)
    // {
    //     RayToLine(mScreenRay[i], rays[i]);
    // }
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
    // std::cout <<"[debug] accel = " << mAccelStructure<<std::endl;
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
            line.mOri = mScreenRay[i].GetOri();
            line.mDest = mScreenRay[i].GetDir() + mScreenRay[i].GetOri();
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

void cPathTracer::RayCastPrimaryRay(std::vector<tLine> & lines, std::vector<tVertex> & pts)const
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

#pragma omp parallel for
    for(int i=0; i < mWidth * mHeight; i++)
    {
        if(i%10 !=0) continue;
        printf("\rprogress: %.3f%", i * 100.0 / (mWidth * mHeight));
        const tRay & ray = mScreenRay[i];
        tVector pos = tVector::Ones() * std::nan("");
        RayCastSingleRay(ray, pos);

        if(pos.hasNaN() == false)
        {
            tLine line;
            tVertex vertex;
            // std::cout << pos.transpose() << std::endl;
            line.mOri = ray.GetOri();
            line.mDest = ray.GetOri() + 10e4 * ray.GetDir();
            vertex.mPos = pos;
            vertex.mColor = tVector(0.5, 0.5, 0.5, 1);

#pragma omp critical
            lines.push_back(line);
#pragma omp critical
            pts.push_back(vertex);
            // std::cout << i << std::endl;
        }
    }
    std::cout << "total tays = " << mHeight * mWidth << std::endl;
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

void cPathTracer::RayCastSingleRay(const tRay & ray, tVector & pt)const
{
    pt = tVector::Ones() * std::nan("");
    std::vector<tFace *> & faces = mSceneMesh->GetFaceList();
    double dist = std::numeric_limits<double>::max();
    if(mAccelStructure == false)
    {
        // std::cout <<" ray ori = " << ray.GetOri().transpose() << std::endl;
        // std::cout <<"ray dir = " << ray.GetDir().transpose() << std::endl;
        for(auto &x : faces)
        {
            tVector p = cMathUtil::RayCast(ray.GetOri(), ray.GetDir(), x->mVertexPtrList[0]->mPos,
                x->mVertexPtrList[1]->mPos,
                x->mVertexPtrList[2]->mPos);
            if(p.hasNaN() == false && (p - ray.GetOri()).norm() < dist)
            {
                pt = p;
                dist = (p - ray.GetOri()).norm();
            }
        }
    }
    else
    {
        // ray-aabb intersect -> ray face intersect
        for(auto & box : mAABBLst)
        {
            if(box.intersect(ray) == true)
            {
                for(auto & x : faces)
                {
                    tVector p = cMathUtil::RayCast(ray.GetOri(), ray.GetDir(), x->mVertexPtrList[0]->mPos,
                        x->mVertexPtrList[1]->mPos,
                        x->mVertexPtrList[2]->mPos);
                    if(p.hasNaN() == false && (p - ray.GetOri()).norm() < dist)
                    {
                        pt = p;
                        dist = (p - ray.GetOri()).norm();
                    }
                }
            }
        }
    }
    
    
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
    for(int i=0; i<3; i++) upper[i] +=1e-3, lower[i]-=1e-3;
    std::cout <<"[debug] upper = " << upper.transpose() <<", lower = " << lower.transpose() << std::endl;
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
    // std::cout << "Face num = " << faces.size() << std::endl;
#pragma omp parallel for
    for(int i=0; i<faces.size(); i++)
    {
        // std::cout <<"dispatch face " << std::endl;
        DispatchFaceToAABB(faces[i]);
    }

    // 查询占空比
    int num = 0;
    std::vector<tAABB>::iterator it = mAABBLst.begin();
    std::cout << "generate aabb num = " << mAABBLst.size() << std::endl;
    for(int i=0; i<mAABBLst.size(); i++)
    {
        if(mAABBLst[i].mFaceId.size() == 0)
        {
            num++;
            it = mAABBLst.erase(it);
        }
        else
        {
            it++;
        }
        
    }
    std::cout << "non-empty aabb num = " << mAABBLst.size() << std::endl;


    // // 绘制所有鸽子
    // for(auto & box : mAABBLst)
    // {
    //     std::vector<tLine> subline;
    //     BuildLinesForBox(subline, box);
    //     glo_lines.insert(glo_lines.end(), subline.begin(), subline.end());
    // }
}

void cPathTracer::DispatchFaceToAABB(const tFace * face)
{
    // std::cout <<"dispatch face = " << face->mFaceId << std::endl;
    bool find_box = false;
    for(int i=0; i<mAABBLst.size(); i++)
    {
        auto &box = mAABBLst[i];
        // std::cout <<"find " << i << std::endl;
        if(true == box.intersect(face))
        {
            find_box = true;
            // std::cout <<"intersect good =  " << i << std::endl;
#pragma omp critical
            box.mFaceId.push_back(face->mFaceId);
        }
    }

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
    // auto FromGroupToId = [](tVector & v, int divide) 
    // { 
    //     return v[0] * divide * divide + v[1] * divide + v[2];
    // };
    // assert(NUM_VERTEX_PER_FACE == 3);
    // tVertex * const *  v_lst = face->mVertexPtrList;
    // tVector lower, upper;
    // mSceneMesh->GetBound(lower, upper);

    // tVector group[3];
    // for(int i=0; i< 3; i++)
    // {
    //     group[i].setZero();
    //     tVector cur_pos = v_lst[i]->mPos;
    //     for(int j=0; j<3; j++) // for x y z 3 dims
    //         group[i][j] = static_cast<int>((cur_pos[j] - lower[j]) / ((upper[j] - lower[j]) / mDivide));
    // }

    // // if 3 points are in the same cube
    // if(cMathUtil::IsSame(group[0], group[1]) && cMathUtil::IsSame(group[0], group[2]))
    // {
    //     int access_id = FromGroupToId(group[0], mDivide);
    //     // std::cout << "id = " << access_id << std::endl;
    //     if(mAABBLst.size() <= access_id)
    //     {
    //         std::cout << "error mAABB list = " << mAABBLst.size() << std::endl;
    //         exit(1);
    //     }
    //     mAABBLst[access_id].mFaceId.push_back(face->mFaceId);
    // }
    // else
    // {
    //     // else, if 3 points are in seperate cubes
    //     // search for all cubes
    //     // std::cout << "aabb list = " << mAABBLst.size( ) << std::endl;
    //     // for(auto & box : mAABBLst)

    //     // std::cout <<"aabb end \n";
    // }
}