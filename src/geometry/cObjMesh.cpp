#include "cObjMesh.h"
#include <fstream>
#include <util/cJsonUtil.hpp>
#define ENABLE_OPENMP

//----------------------cObjMesh------------------------
cObjMesh::cObjMesh(const std::string &filename)
    : cBaseMesh(eMeshType::OBJ, filename)
{
    mEdgeNum = 0;
    mEdgeListExist = false;
    mEdgeList.clear();
    mMaterialList.clear();
    mTexturePtr = nullptr;
}

cObjMesh::~cObjMesh()
{
    mEdgeNum = 0;
    Clear();
}

std::vector<tEdge *> cObjMesh::GetEdgeList() { return mEdgeList; }

void cObjMesh::AddFace(tFace *f)
{
    // add face
    cBaseMesh::AddFace(f);

    // set up shape info
    assert(f->mShapeId >= 0);
    while (mShapeFaceId.size() <= f->mShapeId)
        mShapeFaceId.push_back(std::vector<int>());
    mShapeFaceId[f->mShapeId].push_back(f->mFaceId);
}

void cObjMesh::BuildEdgeList()
{
    // tool func for finding edge
    auto FindEdge = [](const std::vector<tEdge *> &edge_lst,
                       tEdge *new_edge) -> int {
        for (auto &cur_edge : edge_lst)
        {
            if (((new_edge->mOri == cur_edge->mOri) &&
                 (new_edge->mDest == cur_edge->mDest)) ||
                ((new_edge->mOri == cur_edge->mDest) &&
                 (new_edge->mDest == cur_edge->mOri)))
            {
                return cur_edge->mEdgeId;
            }
        }
        return -1;
    };

    // set flag
    mEdgeListExist = true;

    const std::string edge_record = mMeshPath + ".edge";
    std::ifstream fin(edge_record.c_str());
    if (true == fin.fail())
    {
        std::cout << "[log] cObjMesh::BuildEdgeList: begin to build edge table "
                  << edge_record << "..." << std::endl;

        // allocate in advance
        mEdgeList.reserve(mFaceNum);
        tEdge edge_tmp;
#ifdef ENABLE_OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
        for (int i = 0; i < mFaceNum; i++)
        {
            printf("\rbuild edge table %.3f%%", i * 100.0 / mFaceNum);
            tFace *cur_face = mFaceList[i];
            for (int cnt = 0; cnt < 3; cnt++)
            {
                edge_tmp.mOri = mVertexList[cur_face->mVertexIdList[cnt % 3]];
                edge_tmp.mDest =
                    mVertexList[cur_face->mVertexIdList[(cnt + 1) % 3]];
                int cur_id = FindEdge(mEdgeList, &edge_tmp);
                if (cur_id == -1)
                {
                    // set up new edge
                    tEdge *cur_edge = new tEdge();
                    cur_edge->mEdgeId = mEdgeNum++;
                    cur_edge->mOri = edge_tmp.mOri;
                    cur_edge->mDest = edge_tmp.mDest;
                    cur_edge->mOriId = cur_face->mVertexIdList[cnt % 3];
                    cur_edge->mDestId = cur_face->mVertexIdList[(cnt + 1) % 3];

#ifdef ENABLE_OPENMP
#pragma omp critical
#endif
                    mEdgeList.push_back(cur_edge);

                    // set edge id
                    cur_id = cur_edge->mEdgeId;
                }
                cur_face->mEdgeIdList[cnt] = cur_id;
            }
        }

        // write to file
        WriteEdgeList(edge_record);
        // std::cout << "[log] cObjMesh::BuildEdgeList: build edge table succ, write to " << edge_record << std::endl;
    }
    else
    {
        ReadEdgeList(edge_record);
        // std::cout << "[log] cObjMesh::BuildEdgeList: load edge table from " << edge_record << std::endl;
    }

    // std::cout << "[log] edge num in mesh = " << mEdgeNum << std::endl;
}

void cObjMesh::ReadEdgeList(const std::string &path)
{
    Json::Value root;
    cJsonUtil::ParseJson(path, root);

    int edge_num = root["EdgeNum"].asInt();
    int face_num = root["FaceNum"].asInt();
    assert(face_num == mFaceNum);
    mEdgeNum = edge_num;

    // init edge list
    mEdgeList.resize(mEdgeNum);

    Json::Value edge_info = root["EdgeList"], face_info = root["FaceList"];
    for (auto &x : edge_info)
    {
        tEdge *new_edge = new tEdge();
        int edge_id = x["EdgeId"].asInt();
        int ori_id = x["OriVertex"].asInt();
        int dest_id = x["DestVertex"].asInt();
        new_edge->mEdgeId = edge_id;
        new_edge->mOriId = ori_id;
        new_edge->mDestId = dest_id;
        new_edge->mOri = mVertexList[ori_id];
        new_edge->mDest = mVertexList[dest_id];
        mEdgeList[edge_id] = new_edge;
    }

    // set up face info
    for (auto &x : face_info)
    {
        int face_id = x["FaceId"].asInt();
        for (int i = 0; i < NUM_EDGE_PER_FACE; i++)
        {
            std::string name = "EdgeId" + std::to_string(i);
            int sub_edge_id = x[name].asInt();
            assert(sub_edge_id < mEdgeNum);
            mFaceList[face_id]->mEdgeIdList[i] = sub_edge_id;
        }
    }
}

void cObjMesh::WriteEdgeList(const std::string &path)
{
    if (mEdgeListExist == false)
    {
        std::cout << "[error] no edge info exists now!";
        exit(1);
    }
    std::ofstream fout(path);
    Json::Value root, edge_info, face_info;
    root["FaceNum"] = this->mFaceNum;
    root["EdgeNum"] = this->mEdgeNum;

    // write edge list info
    for (auto &x : mEdgeList)
    {
        Json::Value cur_edge;
        cur_edge["EdgeId"] = x->mEdgeId;
        cur_edge["OriVertex"] = x->mOriId;
        cur_edge["DestVertex"] = x->mDestId;
        edge_info.append(cur_edge);
    }
    root["EdgeList"] = edge_info;

    // write face list info
    for (auto &x : mFaceList)
    {
        Json::Value cur_face;
        cur_face["FaceId"] = x->mFaceId;
        for (int i = 0; i < NUM_EDGE_PER_FACE; i++)
        {
            std::string name = "EdgeId" + std::to_string(i);
            cur_face[name] = x->mEdgeIdList[i];
        }
        face_info.append(cur_face);
    }
    root["FaceList"] = face_info;
    fout << root << std::endl;
}

void cObjMesh::GetTexture(unsigned char *&tex, int &width, int &height)
{
    tex = mTexturePtr;
    width = mTexWidth;
    height = mTexHeight;
    // std::cout <<"[debug] cObjMesh::getTex = " << (tex == nullptr) << std::endl;
}

void cObjMesh::SetTexture(unsigned char *tex, int width, int height)
{
    mTexWidth = width;
    mTexHeight = height;
    mTexturePtr = tex;
}

void cObjMesh::PrintInfo()
{
    cBaseMesh::PrintInfo();
    std::cout << ", and the texture is ";
    if (mTexturePtr != nullptr)
        std::cout << "available\n";
    else
        std::cout << "unavailable\n";
}

std::vector<std::pair<tVector, tAABB *>>
cObjMesh::ShapeAnalysis(std::vector<std::vector<int>> &vec)
{
    std::cout << "obj mesh shapre analysis begin\n";
    std::cout << this->mShapeFaceId.size() << std::endl;
    vec = mShapeFaceId;
    std::vector<std::pair<tVector, tAABB *>> BVH_sphere_info;
    auto UpdateBound = [](tAABB &aabb, const tVector &pos) {
        for (int i = 0; i < 3; i++)
        {
            if (aabb.bound[i][0] > pos[i])
                aabb.bound[i][0] = pos[i];
            if (aabb.bound[i][1] < pos[i])
                aabb.bound[i][1] = pos[i];
        }
    };
    for (int shape_id = 0; shape_id < mShapeFaceId.size(); shape_id++)
    {
        // std::cout <<"-----for shape " << shape_id << std::endl;
        const std::vector<int> &id_lst = mShapeFaceId[shape_id];
        tVector center = tVector::Zero();
        // Eigen::Vector2d bound[3];
        // for(int i=0; i<3; i++) bound[i][0] = std::numeric_limits<double>::max(), bound[i][1] = -std::numeric_limits<double>::max();
        tAABB *aabb = new tAABB();
        for (int i = 0; i < id_lst.size(); i++)
        {
            for (int v = 0; v < NUM_VERTEX_PER_FACE; v++)
            {
                auto &cur_v = mFaceList[id_lst[i]]->mVertexPtrList[v];
                center += cur_v->mPos / (NUM_VERTEX_PER_FACE * id_lst.size());
                UpdateBound(*aabb, cur_v->mPos);
            }
        }
        // std::cout <<"center = " << center.transpose() <<std::endl;
        // std::cout <<"x bound = " << aabb->bound[0].transpose() <<std::endl;
        // std::cout <<"y bound = " << aabb->bound[1].transpose() <<std::endl;
        // std::cout <<"z bound = " << aabb->bound[2].transpose() <<std::endl;
        BVH_sphere_info.push_back(std::pair<tVector, tAABB *>(center, aabb));
    }
    return BVH_sphere_info;
    // exit(1);
}

void cObjMesh::AddMaterial(tMaterial *m) { mMaterialList.push_back(m); }

tMaterial *cObjMesh::GetMaterial(int id)
{
    if (id >= mMaterialList.size() || id < 0)
    {
        std::cout << "[error] cObjMesh::GetMaterial exceed size " << id
                  << std::endl;
        exit(1);
    }
    return mMaterialList[id];
}

int cObjMesh::GetMaterialNum() { return mMaterialList.size(); }