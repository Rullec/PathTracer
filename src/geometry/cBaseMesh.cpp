#include "cBaseMesh.h"
#include <util/cJsonUtil.hpp>
#include <iostream>
#include <fstream>
#define ENABLE_OPENMP
// geo method

void BuildLinesForFace(std::vector<tLine>& lines, const tFace * face)
{
    lines.clear();
    tLine line;
    for(int i=0; i<NUM_VERTEX_PER_FACE; i++)
    {
        line.mOri = face->mVertexPtrList[(i + 1) % NUM_VERTEX_PER_FACE]->mPos;
        line.mDest = face->mVertexPtrList[(i + 2) % NUM_VERTEX_PER_FACE]->mPos;
        line.mColor = tVector(1, 0, 0, 1);
        lines.push_back(line);
    }
}

void BuildLinesForBox(std::vector<tLine>& lines, const tAABB & box)
{
    lines.clear();
    tVector low = tVector(box.bound[0][0], box.bound[1][0], box.bound[2][0], 1),
        high = tVector(box.bound[0][1], box.bound[1][1], box.bound[2][1], 1);
    tLine line;
    line.mColor = tVector(0, 0, 0, 1);
    for(int i=0; i< 3; i++)
    {
        // iterate in 3 dims
        line.mOri[i] = low[i];
        line.mDest[i] = high[i];
        int label1 = (i + 1) % 3, label2 = (i+2) % 3;

        // for other 2 dims, we have 4 options
        for(int j=0; j<4; j++)
        {
            line.mOri[label1] = line.mDest[label1] = (j%2 ==0) ? low[label1] : high[label1];
            line.mOri[label2] = line.mDest[label2] = (j/2 ==0) ? low[label2] : high[label2];
            lines.push_back(line);
        }
        
    }
}

bool tAABB::intersect(const tVector & pos)const
{
    double eps = 1e-5;
    for(int i=0; i<3; i++)
    {
        if(pos[i] < bound[i][0] - eps || pos[i] > bound[i][0] + eps)
        {
            return false;
        }
    }
    return true;
}

bool tAABB::intersect(const tRay &ray)const
{
    double t;
    const Eigen::Vector3d & invdir = ray.GetInvDir().segment(0, 3);
    // std::cout <<"inv dir = " << invdir.transpose() << std::endl;
    const Eigen::Vector3d & origin = ray.GetOri().segment(0, 3);
    float t1 = (bound[0][0] - origin.x())*invdir.x();
    float t2 = (bound[0][1] - origin.x())*invdir.x();
    float t3 = (bound[1][0] - origin.y())*invdir.y();
    float t4 = (bound[1][1] - origin.y())*invdir.y();
    float t5 = (bound[2][0] - origin.z())*invdir.z();
    float t6 = (bound[2][1] - origin.z())*invdir.z();

    float tmin = std::max(std::max(std::min(t1, t2), std::min(t3, t4)), std::min(t5, t6));
    float tmax = std::min(std::min(std::max(t1, t2), std::max(t3, t4)), std::max(t5, t6));

    if (tmax < 0)
    {
        t = tmax;
        return false;
    }

    if (tmin > tmax)
    {
        t = tmax;
        return false;
    }

    t = tmin;
    return true;
}

bool tAABB::intersect(const tLine & line)const
{
    tRay r1(line.mOri, line.mDest - line.mOri), r2(line.mDest, line.mOri - line.mDest);
    return intersect(r1) && intersect(r2);
}

void Project(std::vector<tVector> points, const tVector & axis,
        double & pt_min, double & pt_max)
{
    pt_min = std::numeric_limits<double>::max();
    pt_max = -std::numeric_limits<double>::max();
	// std::cout << "min = " <<pt_min <<", max = " << pt_max << std::endl;
    for(auto & p : points)
    {
        double val = axis.dot(p);
        if (val < pt_min) pt_min = val;
        if (val > pt_max) pt_max = val;
    }
}


bool tAABB::intersect(const tFace * face)const
{
	// 上面的不对
    // // first, if any of vertices is in this box, they must intersect
    // int point_outsde = false;
    // for(int i=0; i<NUM_VERTEX_PER_FACE; i++)
    // {
    //     tVector pos = face->mVertexPtrList[i]->mPos;
    //     if(false == intersect(pos))
    //     {
    //         point_outsde = true;
    //         break;
    //     }
    // }
    // if(point_outsde == false)
    // {
    //     // std::cout <<"3 points in true\n";
    //     return true;
    // }

    // // second, if the edges intersect, it is
    // tLine line;
    // for(int i=0; i<3; i++)
    // {
    //     line.mOri = face->mVertexPtrList[(i + 1)%3]->mPos;
    //     line.mDest = face->mVertexPtrList[(i + 2)%3]->mPos;
    //     if(intersect(line))
    //     {

    //         // std::cout <<i << " line in true\n";
    //         return true;
    //     }
    // }
    // return false;
	// 寻找法平面
	std::vector<tVector> triangle_ver(NUM_VERTEX_PER_FACE);
	std::vector<tVector> aabb_ver(8);
	for(int i=0; i< NUM_VERTEX_PER_FACE; i++) triangle_ver[i] = face->mVertexPtrList[i]->mPos;
	for(int x = 0; x< 2; x++)
	for(int y = 0; y< 2; y++)
	for(int z = 0; z< 2; z++)
	{
		aabb_ver[x * 4 + y * 2 + z] = tVector(bound[0][x], bound[1][y], bound[2][z], 1);
		// std::cout <<"Vertices = " << tVector(bound[0][x], bound[1][y], bound[2][z], 1).transpose() <<"\n";
	}
	double triangleMin, triangleMax;
    double boxMin, boxMax;

    // Test the box normals (x-, y- and z-axes)
    tVector boxNormals[3] = {
		tVector(1, 0, 0, 0),
		tVector(0, 1, 0, 0),
		tVector(0, 0, 1, 0),
	};

    for (int i = 0; i < 3; i++)
    {
        tVector n = boxNormals[i];
        Project(triangle_ver, boxNormals[i], triangleMin, triangleMax);
		// std::cout << " i = " << i <<" triangle max min = " << triangleMax <<" " << triangleMin << std::endl;
        if (triangleMax < this->bound[i][0] || triangleMin > this->bound[i][1])
            return false; // No intersection possible.
    }

	// exit(1);
    // Test the triangle normal
	tVector triangle_normal = tVector::Zero();
	{
		tVector v1 = face->mVertexPtrList[1]->mPos - face->mVertexPtrList[0]->mPos,
			v2 = face->mVertexPtrList[2]->mPos - face->mVertexPtrList[1]->mPos;
		triangle_normal = v1.cross3(v2);
		triangle_normal.normalize();
	}
    double triangleOffset = triangle_normal.dot(face->mVertexPtrList[0]->mPos);
    Project(aabb_ver, triangle_normal, boxMin, boxMax);
    if (boxMax < triangleOffset || boxMin > triangleOffset)
        return false; // No intersection possible.

    // Test the nine edge cross-products
    tVector triangleEdges[3] ={
        face->mVertexPtrList[0]->mPos - face->mVertexPtrList[1]->mPos,
		face->mVertexPtrList[1]->mPos - face->mVertexPtrList[2]->mPos,
		face->mVertexPtrList[2]->mPos - face->mVertexPtrList[0]->mPos
    };
    for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
    {
        // The box normals are the same as it's edge tangents
        tVector axis = triangleEdges[i].cross3(boxNormals[j]).normalized();
		// 如果aabb和triangle的这条边平行，那么忽略它，进行下一组判断
		if(std::abs(axis.norm()-1) > 1e-10)
		{
			// std::cout <<"get axis failed = " << axis.transpose() <<", from " << triangleEdges[i].transpose() <<" and " << boxNormals[j].transpose() << std::endl;
			continue;
		}
        Project(aabb_ver, axis, boxMin, boxMax);
        Project(triangle_ver, axis, triangleMin, triangleMax);
        if (boxMax <= triangleMin || boxMin >= triangleMax)
            return false; // No intersection possible
    }

    // No separating axis found.
    return true;
}


//---------------------cBaseMesh------------------------
cBaseMesh::cBaseMesh(const eMeshType type, const std::string & filename):mMeshType(type), mMeshPath(filename)
{
	mFaceNum = 0;
	mVertexNum = 0;
	mFaceList.clear();
	mVertexList.clear();
	mCenterPos = tVector::Zero();

	mUpperBound = std::numeric_limits<double>::min() * tVector::Ones();
	mLowerBound = std::numeric_limits<double>::max() * tVector::Ones();
	mLowerBound[3] = mUpperBound[3] = 0;
}

cBaseMesh::~cBaseMesh()
{

}

void cBaseMesh::AddFace(tFace * f_)
{
	if (!f_ || f_->mFaceId == -1)
	{
		std::cout << "[error] cObjMesh::AddFace illegal input" << std::endl;
		exit(1);
	}
	
	while(f_->mFaceId > mFaceNum - 1)
	{
		mFaceList.push_back(nullptr);
		mFaceNum++;
	}
	
	mFaceList[f_->mFaceId] = f_;
}

void cBaseMesh::AddVertex(tVertex *v_)
{
	if (!v_ || v_->mVertexId == -1)
	{
		std::cout << "[error] cObjMesh::AddVertex illegal input" << std::endl;
		exit(1);
	}

	// for accel, reserve in advance
	mVertexList.reserve(std::max(v_->mVertexId, static_cast<int>(mVertexList.size() + 1)));
	while (v_->mVertexId > mVertexNum - 1)
	{
		mVertexList.push_back(nullptr);
		mVertexNum++;
	}

	if(mVertexList[v_->mVertexId] != nullptr)
	{
		return;
		// auto new_v = v_, old_v = mVertexList[v_->mVertexId];
		// std::cout << "new v pos = " << new_v->mPos.transpose() << std::endl;
		// std::cout << "old v pos = " << old_v->mPos.transpose() << std::endl;
		// std::cout << "new v color = " << new_v->mColor.transpose() << std::endl;
		// std::cout << "old v color = " << new_v->mColor.transpose() << std::endl;
		// std::cout << "new v tex = " << new_v->mTexCoord.transpose() << std::endl;
		// std::cout << "old v tex = " << new_v->mTexCoord.transpose() << std::endl;
		// std::cout <<"--------------------\n";
		
	}
	else
	{
		mVertexList[v_->mVertexId] = v_;

		mCenterPos = (mCenterPos * (mVertexNum - 1) + v_->mPos) / mVertexNum;

		// expand the bounding box
		for(int i=0; i<3; i++)
		{
			if(v_->mPos[i] > mUpperBound[i]) mUpperBound[i] = v_->mPos[i];
			if(v_->mPos[i] < mLowerBound[i]) mLowerBound[i] = v_->mPos[i];
		}
	}
	

}

std::vector<tVertex *> & cBaseMesh::GetVertexList()
{
	return mVertexList;
}

std::vector<tFace * > & cBaseMesh::GetFaceList()
{
	return mFaceList;
}

int cBaseMesh::GetVertexNum()
{
	return mVertexNum;
}

int cBaseMesh::GetFaceNum()
{
	return mFaceNum;
}

tVector cBaseMesh::GetCenter()
{
	return mCenterPos;
}

eMeshType cBaseMesh::GetType()
{
	return mMeshType;
}

void cBaseMesh::PrintInfo()
{
	printf("[log] obj mesh includes %d faces and %d vertices", mFaceNum, mVertexNum);
}


void cBaseMesh::GetBound(tVector & upper, tVector & lower)
{
	upper = mUpperBound;
	lower = mLowerBound;
}

void cBaseMesh::Clear()
{
	for (auto & x : mVertexList) delete x;
	for (auto & x : mFaceList) delete x;
	mFaceNum = 0, mVertexNum = 0;
}

//----------------------cObjMesh------------------------
cObjMesh::cObjMesh(const std::string &filename) :cBaseMesh(eMeshType::OBJ, filename)
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

std::vector<tEdge *> cObjMesh::GetEdgeList()
{
	return mEdgeList;
}

void cObjMesh::BuildEdgeList()
{
	// tool func for finding edge
	auto FindEdge = [](const std::vector<tEdge *> & edge_lst, tEdge * new_edge)->int
	{
		for (auto &cur_edge : edge_lst)
		{
			if (
				((new_edge->mOri == cur_edge->mOri) && (new_edge->mDest == cur_edge->mDest))
				||
				((new_edge->mOri == cur_edge->mDest) && (new_edge->mDest == cur_edge->mOri))
				)
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
		std::cout << "[log] cObjMesh::BuildEdgeList: begin to build edge table " << edge_record << "..." << std::endl;

		// allocate in advance
		mEdgeList.reserve(mFaceNum);
		tEdge edge_tmp;
#ifdef ENABLE_OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
		for (int i = 0; i < mFaceNum; i++)
		{
			printf("\rbuild edge table %.3f%%", i * 100.0/ mFaceNum);
			tFace * cur_face = mFaceList[i];
			for (int cnt = 0; cnt < 3; cnt++)
			{
				edge_tmp.mOri = mVertexList[cur_face->mVertexIdList[cnt % 3]];
				edge_tmp.mDest = mVertexList[cur_face->mVertexIdList[(cnt + 1) % 3]];
				int cur_id = FindEdge(mEdgeList, &edge_tmp);
				if (cur_id == -1)
				{
					// set up new edge
					tEdge * cur_edge = new tEdge();
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

void cObjMesh::ReadEdgeList(const std::string & path)
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
		tEdge * new_edge = new tEdge();
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

void cObjMesh::WriteEdgeList(const std::string & path)
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

void cObjMesh::GetTexture(unsigned char * &tex, int & width, int & height)
{
	tex = mTexturePtr;
	width = mTexWidth;
	height = mTexHeight;
	// std::cout <<"[debug] cObjMesh::getTex = " << (tex == nullptr) << std::endl;
}

void cObjMesh::SetTexture(unsigned char * tex, int width, int height)
{
	mTexWidth = width;
	mTexHeight = height;
	mTexturePtr = tex;
}

void cObjMesh::PrintInfo()
{
	cBaseMesh::PrintInfo();
	std::cout <<", and the texture is ";
	if(mTexturePtr != nullptr) std::cout << "available\n";
	else std::cout <<"unavailable\n";
}

void cObjMesh::AddMaterial(tMaterial * m)
{
	mMaterialList.push_back(m);
}


tMaterial * cObjMesh::GetMaterial(int id)
{
	if(id >= mMaterialList.size() || id < 0)
	{
		std::cout <<"[error] cObjMesh::GetMaterial exceed size " << id << std::endl;
		exit(1);
	}
	return mMaterialList[id];
}


int cObjMesh::GetMaterialNum()
{
	return mMaterialList.size();
}