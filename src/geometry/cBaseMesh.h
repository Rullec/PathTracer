#pragma once
#include <util/cMathUtil.hpp>
#define NUM_VERTEX_PER_FACE 3
#define NUM_EDGE_PER_FACE 3

enum eMeshType {
	OBJ = 0,
	NUM_MESH_TYPE
};

struct tPixel {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	double mX, mY;
	tVector mColor;
	static const int size = 6;
};

struct tLine {
	tVector mOri;
	tVector mDest;
	tVector mColor;
};

/// Mesh Utilities
#include <iostream>
struct tVertex {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	int mVertexId;
	tVector mPos, mNormal, mColor;
	Eigen::Vector2d mTexCoord;
	tVertex() {
		mVertexId = -1;
		mNormal = tVector::Zero();
		mPos = tVector::Zero();
		mUserPtr = nullptr;
		mTexCoord = Eigen::Vector2d::Zero();
	}
	~tVertex()
	{
		//if (mVertexId != -1)std::cout << "Vertex " << mVertexId << " release\n";
	}
	static const int size = 9;

	// attachment info for Z buffer algorithm
	void * mUserPtr;
};

struct tEdge {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	int mEdgeId;
	int mOriId, mDestId;
	tVertex * mOri, * mDest;
	static const int size = 2 * tVertex::size;
	tEdge()
	{
		mOriId = mDestId = -1;
		mEdgeId = -1;
		mOri = mDest = nullptr;
		mUserPtr = nullptr;
	}

	// attach info for Z buffer
	void * mUserPtr;

};

struct tFace {
	int mFaceId;
	int mMaterialId;
	int mVertexIdList[NUM_VERTEX_PER_FACE];
	int mEdgeIdList[NUM_EDGE_PER_FACE];
	tVertex * mVertexPtrList[NUM_VERTEX_PER_FACE];
	tEdge * mEdgePtrList[NUM_EDGE_PER_FACE];

	static const int size = NUM_VERTEX_PER_FACE * tVertex::size;

	tFace() {
		mFaceId = -1;
		mMaterialId = -1;
		memset(mVertexIdList, 0, sizeof(mVertexIdList));
		memset(mEdgeIdList, 0, sizeof(mEdgeIdList));
		memset(mVertexPtrList, 0, sizeof(mVertexPtrList));
		memset(mEdgePtrList, 0, sizeof(mEdgePtrList));
		
		mUserPtr = nullptr;
	}

	tFace(const tFace & f)
	{
		mFaceId = f.mFaceId;
		mMaterialId = f.mMaterialId;
		for(int i=0; i<NUM_VERTEX_PER_FACE; i++)
		{
			mVertexIdList[i] = f.mVertexIdList[i];
			mVertexPtrList[i] = f.mVertexPtrList[i];
		}
		for(int i=0; i<NUM_EDGE_PER_FACE; i++)
		{
			mEdgeIdList[i] = f.mEdgeIdList[i];
			mEdgePtrList[i] = f.mEdgePtrList[i];
		}
		mUserPtr = f.mUserPtr;
	}
	// attach info for Z buffer
	void * mUserPtr;
};

// for an polygin which did not come from a mesh struture
struct tPolygon {
	std::vector<tVector> mVertexLst;
	tPolygon()
	{
		mVertexLst.clear();
	}
};

struct tMaterial{
	std::string mName;
/*
			std::cout <<"diffuse = " << diffuse.transpose() << std::endl;
		std::cout <<"ambient = " << ambient.transpose() << std::endl;
		std::cout <<"specular = " << specular.transpose() << std::endl;
		std::cout <<"transmittance Tf = " << transmittance.transpose() << std::endl;
		std::cout <<"ior Ni = " << ior << std::endl;
		std::cout <<"shininess Ns = " << shininess << std::endl;
		std::cout <<"illum = " << illum << std::endl;
*/
	tVector diffuse, ambient, specular, transmittance;
	double ior, shininess, illum;
	tMaterial()
	{
		diffuse = ambient = specular = transmittance = tVector::Zero();
		ior = shininess = illum = 1;
	}
};


struct tRay{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
public:
    tRay(): mOri(tVector::Zero()), mDir(tVector::Zero()) 
    {

    };
    tRay(const tVector & ori, const tVector & dir){Init(ori, dir);};
    void Init(const tVector & ori, const tVector & dir)
    {
        mOri = ori;
        mDir = dir;
        mInvDir = tVector(1.0/ mDir[0], 1.0/ mDir[1], 1.0/ mDir[2], 0);
        // for(int i=0; i<3; i++) mInvDir[i] *= (mDir[0] > 0 ? 1 : -1);
        for(int i=0; i<3; i++) if(std::isinf(mInvDir[i])) mInvDir[i] = 1e10 * (mDir[0] > 0 ? 1 : -1);
        // for(int i=0; i<3; i++) mInvDir[i] *= (mDir[i] < 0);
        // std::cout << mInvDir.transpose() << std::endl;
    }
    tVector GetOri() const{return mOri;}
    tVector GetDir() const{return mDir;}
    tVector GetInvDir() const {return mInvDir;}
    // const bool * GetSign() const{return sign;}

private:
    tVector mOri;
    tVector mDir;
    // bool sign[3];
    tVector mInvDir;
};

struct tAABB{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Eigen::Vector2d bound[3]; 
    // bound[0]:x_bound, bound[1]:y_bound, bound[2]:z_bound
    // bound[i][0] lower bound, bound[i][1] upper bound
    std::vector<int> mFaceId;
    tAABB()
    {
        bound[0] = bound[1] = bound[2] = Eigen::Vector2d::Zero();
        mFaceId.clear();
    };
    bool intersect(const tVector & pos)const;
    bool intersect(const tRay & ray)const;
    bool intersect(const tLine & line)const;
    bool intersect(const tFace * face)const;
};

// methods

void BuildLinesForFace(std::vector<tLine>& lines, const tFace * face);
void BuildLinesForBox(std::vector<tLine>& lines, const tAABB & box);

class cBaseMesh
{
public:
	cBaseMesh(const eMeshType type_, const std::string &);
	virtual ~cBaseMesh() = 0;

	// add face
	virtual void AddFace(tFace *);
	virtual void AddVertex(tVertex *);

	// get method
	virtual std::vector<tVertex *> &GetVertexList();
	virtual std::vector<tFace * > &GetFaceList();
	virtual int GetVertexNum();
	virtual int GetFaceNum();
	tVector GetCenter();
	eMeshType GetType();
	
	// print method
	virtual void PrintInfo();
	void GetBound(tVector & upper, tVector & lower);
	
protected:
	const eMeshType mMeshType;			// which type it belongs to?
	const std::string mMeshPath;
	int mFaceNum, mVertexNum;	// face cnt, vertices cnt
	std::vector<tFace *> mFaceList;
	std::vector<tVertex *> mVertexList;
	tVector mUpperBound, mLowerBound;	// upper & lower bound for bouding box of this mesh

	tVector mCenterPos;					// the shape center of this 3D mesh
	void Clear();
};

class cObjMesh :public cBaseMesh {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	cObjMesh(const std::string & filename);
	~cObjMesh();
	std::vector<tEdge *> GetEdgeList();
	void BuildEdgeList();
	void GetTexture(unsigned char * &, int &w, int &h);
	void SetTexture(unsigned char *, int w, int h);
	
	void AddMaterial(tMaterial *);
	tMaterial * GetMaterial(int id);
	int GetMaterialNum();
	virtual void PrintInfo() override ;
protected:

	int mEdgeNum;
	bool mEdgeListExist;
	std::vector<tEdge *> mEdgeList;

	// texture 
	int mTexWidth, mTexHeight;
	unsigned char * mTexturePtr;

	// material storage
	std::vector<tMaterial *> mMaterialList;

	// tool methods
	void ReadEdgeList(const std::string & path);
	void WriteEdgeList(const std::string & path);
};