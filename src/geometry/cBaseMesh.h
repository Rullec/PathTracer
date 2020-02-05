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