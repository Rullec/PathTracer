// raycast class
#pragma once
#include <util/cJsonUtil.hpp>
#include <util/cMathUtil.hpp>
#include <iostream>
#include <string>

enum eAccelType{
    NONE,
    AABB,
    OCTREE,
    NUM_ACCEL_TYPE,
    INVALID_ACCEL_TYPE
};

const std::string gDescAccelType[] = {
    "None",
    "AABB",
    "OCTREE"
};

class cBaseMesh;
struct tRay;
struct tFace;
struct tAABB;
struct tOctreeNode;

class cAccelStruct{
public:
    cAccelStruct(eAccelType Type);
    virtual void Init(std::shared_ptr<cBaseMesh>  mesh);
    virtual tFace * RayCast(const tRay & ray, tVector & pt);
    virtual bool VisTest(const tVector & p1, const tVector & p2);
    eAccelType GetType();

protected:
    eAccelType mType;
    std::shared_ptr<cBaseMesh> mMesh;
};

class cAABB: public cAccelStruct{
public:
    cAABB(int);
    virtual void Init(std::shared_ptr<cBaseMesh>  mesh) override final;
    virtual tFace * RayCast(const tRay & ray, tVector & pt) override final;
    virtual bool VisTest(const tVector & p1, const tVector & p2) override final;
protected:
    std::vector<tAABB> mAABBLst;
    int mDivide;
};

class cOctree : public cAccelStruct{
public:
    cOctree(int capacity);
    virtual void Init(std::shared_ptr<cBaseMesh>  mesh) override final;
    virtual tFace * RayCast(const tRay & ray, tVector & pt) override final;
    virtual bool VisTest(const tVector & p1, const tVector & p2) override final;    
    void PrintTree();
    
protected:
    tOctreeNode * root;
    int mCapacity;
    int mNumNodes;
    bool AddFace(tOctreeNode * cur_node, tFace *);
};

std::shared_ptr<cAccelStruct> BuildAccelStruct(Json::Value & accel_struct);