#include "cAccelStruct.hpp"
#include "util/cJsonUtil.hpp"
#include <geometry/cBaseMesh.h>
#include <util/cMathUtil.hpp>
#define ENABLE_OPENMP
static int node_num = 0;
// Accel struct
cAccelStruct::cAccelStruct(eAccelType type)
{
    mType = type;
    mMesh = nullptr;
}

eAccelType cAccelStruct::GetType() { return mType; }

void cAccelStruct::Init(std::shared_ptr<cBaseMesh> mesh)
{
    if (nullptr == mesh)
    {
        std::cout << "[error] cAccelStruct::Init mesh null!\n";
        exit(1);
    }
    mMesh = mesh;
    // std::cout <<"[debug] Accel struct Init\n";
    // exit(1);
}

tFace *cAccelStruct::RayCast(const tRay &ray, tVector &pt)
{
    if (nullptr == mMesh)
    {
        std::cout << "[error] cAccelStruct::Inquir mesh null!\n";
        exit(1);
    }
    std::vector<tFace *> &face_lst = mMesh->GetFaceList();
    pt = tVector::Ones() * std::nan("");
    tFace *target_face = nullptr;
    double dist = std::numeric_limits<double>::max();
    for (auto &x : face_lst)
    {
        tVector p = cMathUtil::RayCast(
            ray.GetOri(), ray.GetDir(), x->mVertexPtrList[0]->mPos,
            x->mVertexPtrList[1]->mPos, x->mVertexPtrList[2]->mPos);
        if (p.hasNaN() == false && (p - ray.GetOri()).norm() < dist &&
            (p - ray.GetOri()).norm() > 1e-6)
        {
            pt = p;
            dist = (p - ray.GetOri()).norm();
            target_face = x;
        }
    }
    // std::cout <<"cAccelStruct begin judge\n" ;
    return target_face;
}

bool cAccelStruct::VisTest(const tVector &p1, const tVector &p2)
{
    double dist = (p1 - p2).norm();
    double cur_min_dist = dist;
    tVector light_dir = (p2 - p1).normalized();
    std::vector<tFace *> &full_faces = mMesh->GetFaceList();
    for (auto &x : full_faces)
    {
        double p = cMathUtil::RayCastT(
            p1, light_dir, x->mVertexPtrList[0]->mPos,
            x->mVertexPtrList[1]->mPos, x->mVertexPtrList[2]->mPos);
        assert(std::isnan(p) || p > -1e-6); // if p <=0, illegal

        // p > 1e-6: in case of the intersection is with light it self
        if (std::isnan(p) == false && p < cur_min_dist - 1e-6 && p > 1e-6)
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
            if (cur_min_dist < dist)
                return false;
        }
    }

    return true;
}
// AABB
cAABB::cAABB(int divide) : cAccelStruct(eAccelType::AABB)
{
    assert(divide >= 1);
    mDivide = divide;
}

void cAABB::Init(std::shared_ptr<cBaseMesh> mesh)
{
    // std::cout <<"AABB accel struct init \n";
    mMesh = mesh;

    // build aabb
    tVector upper, lower;
    mMesh->GetBound(upper, lower);
    for (int i = 0; i < 3; i++)
        upper[i] += 1e-5, lower[i] -= 1e-5;
    // std::cout <<"[debug] upper = " << upper.transpose() <<", lower = " << lower.transpose() << std::endl;

    // divide the whole space
    tAABB box;
    mAABBLst.clear();
    int id[3] = {0, 0, 0};
    for (id[0] = 0; id[0] < mDivide; id[0]++)
        for (id[1] = 0; id[1] < mDivide; id[1]++)
            for (id[2] = 0; id[2] < mDivide; id[2]++)
            {
                for (int i = 0; i < 3; i++)
                {
                    box.bound[i][0] =
                        lower[i] + (upper - lower)[i] / mDivide * id[i];
                    box.bound[i][1] =
                        lower[i] + (upper - lower)[i] / mDivide * (id[i] + 1);
                }
                mAABBLst.push_back(box);
            }

    // dispatch all faces to these AABB
    std::vector<tFace *> &faces = mMesh->GetFaceList();
#ifdef ENABLE_OPENMP
#pragma omp parallel for
#endif
    for (int i = 0; i < faces.size(); i++)
    {
        auto &face = faces[i];
        bool find_box = false;
        for (int i = 0; i < mAABBLst.size(); i++)
        {
            auto &box = mAABBLst[i];
            // std::cout <<"find " << i << std::endl;
            if (true == box.intersect(face))
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
        if (false == find_box)
        {
            std::cout << "[error] find box for face failed" << std::endl;
            for (int i = 0; i < NUM_VERTEX_PER_FACE; i++)
                std::cout << face->mVertexPtrList[i]->mPos.transpose()
                          << std::endl;

            std::cout << "all boxes info\n";
            for (auto &box : mAABBLst)
            {
                std::cout << "-----------\n";
                std::cout << "box x range = " << box.bound[0].transpose()
                          << std::endl;
                std::cout << "box y range = " << box.bound[1].transpose()
                          << std::endl;
                std::cout << "box z range = " << box.bound[2].transpose()
                          << std::endl;
            }
            exit(1);
        }
    }

    // remove all empty boxes
    mAABBLst.erase(std::remove_if(mAABBLst.begin(), mAABBLst.end(),
                                  [](const tAABB &box) {
                                      return box.mFaceId.size() == 0;
                                  }),
                   mAABBLst.end());
    // exit(1);
}

tFace *cAABB::RayCast(const tRay &ray, tVector &pt)
{
    if (mMesh == nullptr)
    {
        std::cout << "[error] cAABB::Inquiry error\n";
        exit(1);
    }
    std::vector<tFace *> &face_lst = mMesh->GetFaceList();
    pt = tVector::Ones() * std::nan("");
    tFace *target_face = nullptr;
    double dist = std::numeric_limits<double>::max();
    for (auto &box : mAABBLst)
    {
        if (box.intersect(ray) == false)
            continue;
        for (auto &id : box.mFaceId)
        {
            auto &x = face_lst[id];
            tVector p = cMathUtil::RayCast(
                ray.GetOri(), ray.GetDir(), x->mVertexPtrList[0]->mPos,
                x->mVertexPtrList[1]->mPos, x->mVertexPtrList[2]->mPos);
            if (p.hasNaN() == false && (p - ray.GetOri()).norm() < dist &&
                (p - ray.GetOri()).norm() > 1e-6)
            {
                pt = p;
                dist = (p - ray.GetOri()).norm();
                target_face = x;
            }
        }
    }
    return target_face;
}

bool cAABB::VisTest(const tVector &p1, const tVector &p2)
{
    bool mAccelStructure = true;
    double dist = (p1 - p2).norm();
    double cur_min_dist = dist;
    tVector light_dir = (p2 - p1).normalized();
    std::vector<tFace *> &full_faces = mMesh->GetFaceList();
    tLine cur_line;
    cur_line.mOri = p1;
    cur_line.mDest = p2;
    for (auto &box : mAABBLst)
    {
        if (box.intersect(cur_line) == false)
            continue;
        for (auto &id : box.mFaceId)
        {
            auto &x = full_faces[id];
            double p = cMathUtil::RayCastT(
                p1, light_dir, x->mVertexPtrList[0]->mPos,
                x->mVertexPtrList[1]->mPos, x->mVertexPtrList[2]->mPos);
            assert(std::isnan(p) || p > 0);
            if (std::isnan(p) == false && p < cur_min_dist - 1e-6 && p > 1e-6)
            {
                cur_min_dist = p;

                // it's invisible if here is an intersect
                if (cur_min_dist < dist)
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

    return true;
}

// OcTree
struct tOctreeNode
{
public:
    tOctreeNode(const Eigen::Vector2d *bound, int node_id);
    // ~tOcTreeNode(){for(int i=0; i<8; i++) delete Children[i];};

    struct tOctreeNode *Children[2][2][2];
    struct tAABB mbox;
    bool IsLeaf;
    int id;
    void split(std::shared_ptr<cBaseMesh> mesh, int capacity);
};

tOctreeNode::tOctreeNode(const Eigen::Vector2d *bound, int node_id)
{
    assert(node_id >= 0);
    for (int i = 0; i < 8; i++)
        Children[i / 4][(i % 4) / 2][i % 2] = nullptr;
    mbox.mFaceId.clear();
    for (int i = 0; i < 3; i++)
        mbox.bound[i] = bound[i];
    IsLeaf = true;
    id = node_id;
}

void tOctreeNode::split(std::shared_ptr<cBaseMesh> mesh, int capacity)
{
    IsLeaf = false;
    Eigen::Vector2d bound[3];

    double x_wdith = mbox.bound[0][1] - mbox.bound[0][0];
    double y_wdith = mbox.bound[1][1] - mbox.bound[1][0];
    double z_wdith = mbox.bound[2][1] - mbox.bound[2][0];
    double x_st = mbox.bound[0][0];
    double y_st = mbox.bound[1][0];
    double z_st = mbox.bound[2][0];

    // std::cout << x_st <<" " << y_st <<" " << z_st << std::endl;
    // std::cout <<x_wdith <<" " << y_wdith <<" " << z_wdith<<std::endl;
    // void split();
    for (int x = 0; x < 2; x++)
    {
        bound[0][0] = x_st + x / 2.0 * x_wdith;
        bound[0][1] = x_st + (x + 1) / 2.0 * x_wdith;
        for (int y = 0; y < 2; y++)
        {
            bound[1][0] = y_st + y / 2.0 * y_wdith;
            bound[1][1] = y_st + (y + 1) / 2.0 * y_wdith;
            for (int z = 0; z < 2; z++)
            {
                bound[2][0] = z_st + z / 2.0 * z_wdith;
                bound[2][1] = z_st + (z + 1) / 2.0 * z_wdith;
                // int child_id = id * 8 + x * 4 + y * 2 + z + 1;
                // std::cout <<"add child id = " << child_id << std::endl;
                Children[x][y][z] = new tOctreeNode(bound, node_num++);

                // // check print out
                // std::cout <<"for " << x <<" " << y << " " << z << std::endl;
                // char label[3] = {'x', 'y', 'z'};
                // for(int i=0; i<3; i++)
                // {
                //     std::cout <<"bound " << label[i] << " " << Children[x][y][z]->mbox.bound[i][0] <<" to " <<  Children[x][y][z]->mbox.bound[i][1] << std::endl;
                // }
            }
        }
    }

    // 对当前所有face, 都要放进来
    std::vector<tFace *> faces = mesh->GetFaceList();
    for (auto &f_id : mbox.mFaceId)
    {
        bool add_face_succ = false;
        for (int x = 0; x < 2; x++)
            for (int y = 0; y < 2; y++)
                for (int z = 0; z < 2; z++)
                {
                    if (Children[x][y][z]->mbox.intersect(faces[f_id]))
                    {
                        add_face_succ = true;
                        Children[x][y][z]->mbox.mFaceId.push_back(f_id);
                        if (Children[x][y][z]->mbox.mFaceId.size() == capacity)
                        {
                            Children[x][y][z]->split(mesh, capacity);
                        }
                        assert(Children[x][y][z]->mbox.mFaceId.size() <
                               capacity);
                    }
                }

        if (add_face_succ == false)
        {
            std::cout << "[error] tOctreeNode::split add face " << f_id
                      << " in node " << this->id << " failed\n";
            tVertex **v = faces[f_id]->mVertexPtrList;
            std::cout << "[info] face " << f_id << " is "
                      << v[0]->mPos.transpose() << ", "
                      << v[1]->mPos.transpose() << ", "
                      << v[2]->mPos.transpose() << std::endl;
            std::cout << "[info] this node";
            std::cout << "x " << this->mbox.bound[0].transpose() << std::endl;
            std::cout << "y " << this->mbox.bound[1].transpose() << std::endl;
            std::cout << "z " << this->mbox.bound[2].transpose() << std::endl;

            exit(1);
        }
    }
    mbox.mFaceId.clear();

    // exit(1);
}

cOctree::cOctree(int capacity) : cAccelStruct(eAccelType::OCTREE)
{
    // mMaxDepth = max_depth;
    assert(capacity >= 1);
    root = nullptr;
    mCapacity = capacity;
    mNumNodes = 0;
}
#include <util/cFileUtil.h>
void cOctree::Init(std::shared_ptr<cBaseMesh> mesh)
{
    assert(mesh != nullptr);
    mMesh = mesh;

    const std::string path = mesh->GetMeshPath() + ".octree";

    if (cFileUtil::ExistsFile(path) == false)
    {
        // 1. get boundary
        tVector upper, lower;
        mMesh->GetBound(upper, lower);
        for (int i = 0; i < 3; i++)
            upper[i] += 1e-5, lower[i] -= 1e-5;

        // 2. create root
        Eigen::Vector2d bound[3];
        for (int i = 0; i < 3; i++)
            bound[i][0] = lower[i], bound[i][1] = upper[i];
        root = new tOctreeNode(bound, node_num++);
        std::vector<tFace *> faces = mMesh->GetFaceList();
        for (auto &face : faces)
        {
            // put cur face into this tree
            AddFace(root, face);
        }

        // 3.
        WriteOctree(path);
    }
    else
    {
        std::cout << "[debug] cOctree::Init begin read octree\n";
        ReadOctree(path);
        std::cout << "[debug] cOctree::Init end read octree\n";
    }
    // exit(1);
    // PrintTree();
}

bool cOctree::AddFace(tOctreeNode *cur_node, tFace *face)
{
    // tOctreeNode * cur_node = root;
    assert(cur_node != nullptr);
    bool add_succ = false;

    if (true == cur_node->mbox.intersect(face))
    {
        // 若相交，加入
        if (cur_node->IsLeaf == true)
        {
            if (cur_node->mbox.mFaceId.size() < mCapacity)
            {
                // 如果还没满
                cur_node->mbox.mFaceId.push_back(face->mFaceId);
                add_succ = true;

                // 这次塞满了
                if (cur_node->id == 4)
                {
                    std::cout << "node 4 " << cur_node->mbox.mFaceId.size()
                              << std::endl;
                }

                if (cur_node->mbox.mFaceId.size() == mCapacity)
                {
                    // std::cout <<"split ! node " << cur_node->id << std::endl;
                    // 分裂，将当前所有face id派生到下面的盒子去
                    cur_node->split(mMesh, mCapacity);
                    assert(cur_node->mbox.mFaceId.size() == 0);
                }
            }
            else
            {
                // 如果满了, 异常情况
                std::cout << "exception case\n";
                std::cout << "node " << cur_node->id
                          << " is leaf = " << cur_node->IsLeaf << std::endl;
                std::cout << "size = " << cur_node->mbox.mFaceId.size() << ", ";
                PrintTree();
                exit(1);
            }
        }
        else
        {
            // for 8 children leaves
            for (int x = 0; x < 2; x++)
                for (int y = 0; y < 2; y++)
                    for (int z = 0; z < 2; z++)
                    {
                        add_succ |= AddFace(cur_node->Children[x][y][z], face);
                    }
        }
    }

    return add_succ;
}

#include <queue>
bool cOctree::WriteOctree(const std::string &path)
{
    std::cout << "begin to write octree to " << path << std::endl;
    if (cFileUtil::ExistsFile(path))
        cFileUtil::ClearFile(path);
    std::ofstream fout(path);
    std::queue<tOctreeNode *> Q;
    Q.push(root);
    tOctreeNode *cur_node = nullptr;
    while (Q.size() != 0)
    {
        cur_node = Q.front();
        Q.pop();
        fout << cur_node->id << " " << cur_node->IsLeaf << " ";
        // 然后输出 bound
        for (int i = 0; i < 3; i++)
            fout << cur_node->mbox.bound[i].transpose() << " ";
        if (cur_node->IsLeaf == false)
        {
            // 是中间
            for (int x = 0; x < 2; x++)
                for (int y = 0; y < 2; y++)
                    for (int z = 0; z < 2; z++)
                        fout << cur_node->Children[x][y][z]->id << " ",
                            Q.push(cur_node->Children[x][y][z]);
        }
        else
        {
            for (int i = 0; i < cur_node->mbox.mFaceId.size(); i++)
                fout << cur_node->mbox.mFaceId[i] << " ";
        }
        fout << std::endl;
    }
    std::cout << "end to write octree to " << path << std::endl;
    return true;
}

// #include <fstream>
#include <sstream>
bool cOctree::ReadOctree(const std::string &path)
{
    std::ifstream fin(path);
    std::string line;
    int node_id;
    bool is_leaf;
    int tmp_value;
    const int buffer_num = 1000000;
    tOctreeNode *value_list[buffer_num];
    memset(value_list, 0, sizeof(tOctreeNode *) * buffer_num);
    Eigen::Vector2d bound[3];
    // int i=0;
    while (fin.good())
    {
        node_id = -1;
        // std::cout << i++ << std::endl;
        std::getline(fin, line);
        std::stringstream ss(line);

        // node id and leaf or not?
        ss >> node_id >> is_leaf;
        // std::cout << node_id << std::endl;
        if (node_id == -1)
            break;

        if (node_id > buffer_num)
        {
            std::cout << "cOctree::ReadOctree: buffer exceed\n";
            exit(1);
        }

        // bound box for this box
        ss >> bound[0][0] >> bound[0][1] >> bound[1][0] >> bound[1][1] >>
            bound[2][0] >> bound[2][1];
        if (value_list[node_id] == 0)
        {
            value_list[node_id] = new tOctreeNode(bound, node_id);
        }
        else
        {
            for (int i = 0; i < 3; i++)
                value_list[node_id]->mbox.bound[i] = bound[i];
        }
        value_list[node_id]->IsLeaf = is_leaf;

        if (false == is_leaf)
        {
            // 不是叶子，读取8个儿子id，并且创建儿子
            int times = 0;
            for (int x = 0; x < 2; x++)
                for (int y = 0; y < 2; y++)
                    for (int z = 0; z < 2; z++)
                    {
                        assert(ss.good());
                        ss >> tmp_value;
                        if (tmp_value > buffer_num)
                        {
                            std::cout << "cOctree::ReadOctree: buffer exceed\n";
                            exit(1);
                        }
                        if (value_list[tmp_value] == 0)
                        {
                            // 这里的bound是乱设置的，只是为了满足需求而已。
                            value_list[tmp_value] =
                                new tOctreeNode(bound, tmp_value);
                        }
                        value_list[node_id]->Children[x][y][z] =
                            value_list[tmp_value];
                    }
        }
        else
        {
            // 是叶子，读取本叶子里面所有的face id，存放
            while (ss.good())
            {
                tmp_value = -1;
                ss >> tmp_value;
                if (-1 == tmp_value)
                    break;
                value_list[node_id]->mbox.mFaceId.push_back(tmp_value);
            }
        }
    }
    root = value_list[0];
    // exit(1);
    // WriteOctree("test.octree");
    return true;
}

#include <queue>
tFace *cOctree::RayCast(const tRay &ray, tVector &pt)
{
    std::vector<tFace *> &face_lst = mMesh->GetFaceList();
    pt = tVector::Ones() * std::nan("");
    tFace *target_face = nullptr, *cur_face = nullptr;

    double dist = std::numeric_limits<double>::max();

    // 树的bfs，收入所有可疑地叶子结点
    // 对树进行dfs
    std::queue<tOctreeNode *> Q;
    Q.push(root);
    tOctreeNode *cur_node = nullptr;
    while (Q.size() != 0)
    {
        cur_node = Q.front();
        Q.pop();

        // if it is leaf
        if (cur_node->IsLeaf == true)
        {
            if (true == cur_node->mbox.intersect(ray))
            {
                // leaf intersected, iterate on all faces
                for (auto &face_id : cur_node->mbox.mFaceId)
                {
                    cur_face = face_lst[face_id];
                    tVector p =
                        cMathUtil::RayCast(ray.GetOri(), ray.GetDir(),
                                           cur_face->mVertexPtrList[0]->mPos,
                                           cur_face->mVertexPtrList[1]->mPos,
                                           cur_face->mVertexPtrList[2]->mPos);
                    if (p.hasNaN() == false &&
                        (p - ray.GetOri()).norm() < dist &&
                        (p - ray.GetOri()).norm() > 1e-6)
                    {
                        pt = p;
                        dist = (p - ray.GetOri()).norm();
                        target_face = cur_face;
                    }
                }
            }
        }
        else if (true == cur_node->mbox.intersect(ray))
        {
            // it is not leaf, but they are intersected with each other
            for (int x = 0; x < 2; x++)
                for (int y = 0; y < 2; y++)
                    for (int z = 0; z < 2; z++)
                    {
                        Q.push(cur_node->Children[x][y][z]);
                    }
        }
    }
    return target_face;
}

bool cOctree::VisTest(const tVector &p1, const tVector &p2)
{
    double dist = (p1 - p2).norm();
    double cur_min_dist = dist;
    tVector light_dir = (p2 - p1).normalized();
    std::vector<tFace *> &full_faces = mMesh->GetFaceList();
    tLine cur_line;
    cur_line.mOri = p1;
    cur_line.mDest = p2;

    // 树的bfs，收入所有可疑地叶子结点
    // 对树进行dfs
    std::queue<tOctreeNode *> Q;
    Q.push(root);
    tFace *cur_face;
    tOctreeNode *cur_node = nullptr;
    while (Q.size() != 0)
    {
        cur_node = Q.front();
        Q.pop();

        // if it is leaf
        if (cur_node->IsLeaf == true)
        {
            if (true == cur_node->mbox.intersect(cur_line))
            {
                // leaf intersected, iterate on all faces
                for (auto &face_id : cur_node->mbox.mFaceId)
                {
                    cur_face = full_faces[face_id];
                    double p = cMathUtil::RayCastT(
                        p1, light_dir, cur_face->mVertexPtrList[0]->mPos,
                        cur_face->mVertexPtrList[1]->mPos,
                        cur_face->mVertexPtrList[2]->mPos);
                    assert(std::isnan(p) || p > -1e-6); // if p <=0, illegal

                    // p > 1e-6: in case of the intersection is with light it self
                    if (std::isnan(p) == false && p < cur_min_dist - 1e-6 &&
                        p > 1e-6)
                    {
                        cur_min_dist = p;
                        if (cur_min_dist < dist)
                            return false;
                    }
                }
            }
        }
        else if (true == cur_node->mbox.intersect(cur_line))
        {
            // it is not leaf, but they are intersected with each other
            for (int x = 0; x < 2; x++)
                for (int y = 0; y < 2; y++)
                    for (int z = 0; z < 2; z++)
                    {
                        Q.push(cur_node->Children[x][y][z]);
                    }
        }
    }
    return true;
}

void cOctree::PrintTree()
{
    std::cout << "----------print tree info------------\n";
    std::queue<tOctreeNode *> Q;
    Q.push(root);
    tFace *cur_face;
    tOctreeNode *cur_node = nullptr;
    while (Q.size() != 0)
    {
        cur_node = Q.front();
        Q.pop();
        std::cout << "\nnode " << cur_node->id << " : ";
        if (cur_node->IsLeaf == false)
        {
            std::cout << " has child ";
            for (int x = 0; x < 2; x++)
                for (int y = 0; y < 2; y++)
                    for (int z = 0; z < 2; z++)
                    {
                        Q.push(cur_node->Children[x][y][z]);
                        std::cout << cur_node->Children[x][y][z]->id << " ";
                    }
        }
        else
        {
            std::cout << "is child, include face ";
            for (auto &id : cur_node->mbox.mFaceId)
                std::cout << id << " ";
        }
    }

    // std::cout <<"print over\n";
}

// build
std::shared_ptr<cAccelStruct> BuildAccelStruct(Json::Value &accel_struct_json)
{
    assert(accel_struct_json.isNull() == false);
    /*
		"RayCastAccel" : 
		{
			"Algo" : "AABB",
			"Algo_bak" : "BVH",
			"Subdivide" : 2,
		}
    */
    const std::string type_str = accel_struct_json["Type"].asString();
    eAccelType type = eAccelType::INVALID_ACCEL_TYPE;
    for (int i = 0; i < eAccelType::NUM_ACCEL_TYPE; i++)
    {
        // std::cout << type_str <<" " << gDescAccelType[i] << std::endl;
        if (type_str == gDescAccelType[i])
        {
            type = static_cast<eAccelType>(i);
            break;
        }
    }

    std::shared_ptr<cAccelStruct> accel_struct = nullptr;
    switch (type)
    {
    // case eAccelType::INVALID_ACCEL_TYPE: std::cout <<"[error] BuildAccelStruct invalid type " << type_str << std::endl;exit(1); break;
    case eAccelType::NONE:
        accel_struct =
            std::shared_ptr<cAccelStruct>(new cAccelStruct(eAccelType::NONE));
        break;
    case eAccelType::AABB:
        accel_struct = std::shared_ptr<cAABB>(
            new cAABB(accel_struct_json["Subdivide"].asInt()));
        break;
    case eAccelType::OCTREE:
        accel_struct = std::shared_ptr<cOctree>(
            new cOctree(accel_struct_json["Capacity"].asInt()));
        break;
    default:
        std::cout << "[error] BuildAccelStruct invalid type " << type_str
                  << std::endl;
        exit(1);
        break;
        break;
    }

    std::cout << "[debug] create accel struct tpye = " << type_str << std::endl;
    // exit(1);

    return accel_struct;
}