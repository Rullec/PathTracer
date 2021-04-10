#pragma once
#include "cBaseMesh.h"

class cObjMesh : public cBaseMesh
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    cObjMesh(const std::string &filename);
    ~cObjMesh();
    std::vector<tEdge *> GetEdgeList();
    void BuildEdgeList();
    void GetTexture(unsigned char *&, int &w, int &h);
    void SetTexture(unsigned char *, int w, int h);

    void AddMaterial(tMaterial *);
    virtual void AddFace(tFace *) override final;

    tMaterial *GetMaterial(int id);
    int GetMaterialNum();
    virtual void PrintInfo() override;
    std::vector<std::pair<tVector, tAABB *>>
    ShapeAnalysis(std::vector<std::vector<int>> &vec);

protected:
    int mEdgeNum;
    bool mEdgeListExist;
    std::vector<tEdge *> mEdgeList;

    // texture
    int mTexWidth, mTexHeight;
    unsigned char *mTexturePtr;

    // material storage
    std::vector<tMaterial *> mMaterialList;

    // face id list per shape
    std::vector<std::vector<int>> mShapeFaceId;

    // tool methods
    void ReadEdgeList(const std::string &path);
    void WriteEdgeList(const std::string &path);
};