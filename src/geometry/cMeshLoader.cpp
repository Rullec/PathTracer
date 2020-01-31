#include "cMeshLoader.h"
#include <iostream>
#define TINYOBJLOADER_IMPLEMENTATION 
#include <tinyobj/tiny_obj_loader.h>
using namespace std;

cMeshLoader::cMeshLoader()
{

}

std::shared_ptr<cBaseMesh> cMeshLoader::Load(const std::string & name, eMeshType type)
{
	std::shared_ptr<cBaseMesh> mesh_ = nullptr;
	switch (type)
	{
	case OBJ:
	{
		std::shared_ptr<cObjMesh> x = LoadObj(name);
		mesh_ = std::dynamic_pointer_cast<cBaseMesh> (x);
	}
		break;
	default:
	{
		std::cout << "[error] cMeshLoader::Load: Unsupported mesh type " << type << std::endl;
		exit(1);
	}
		break;
	}

	return mesh_;
}

std::shared_ptr<cObjMesh> cMeshLoader::LoadObj(const std::string & name)
{
	auto obj_ = (std::shared_ptr<cObjMesh>)(new cObjMesh(name));

	tinyobj::attrib_t attrib;
	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::material_t> materials;
	if (false == tinyobj::LoadObj(&attrib, &shapes, &materials, nullptr, nullptr, name.c_str()))
	{
		std::cout << "[error] cMeshLoader::LoadObj " << name << " failed " << std::endl;
		exit(1);
	}

	for (size_t s = 0; s < shapes.size(); s++) {
		//std::cout << "shape size = " << shapes.size() << std::endl;

		// Loop over faces(polygon)
		size_t index_offset = 0;
		for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
			int fv = shapes[s].mesh.num_face_vertices[f];
			if (NUM_VERTEX_PER_FACE != fv)
			{
				std::cout << "[error] cMeshLoader::LoadObj: didn't support faces with " << fv << " vertices" << std::endl;
				exit(1);
			}

			tFace *cur_face = new tFace();
			cur_face->mFaceId = f;
			obj_->AddFace(cur_face);

			// Loop over vertices in the face.
			for (size_t v = 0; v < fv; v++) {
				// access to vertex
				tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
				//std::cout << "vertex = " << idx.vertex_index << std::endl;
				tinyobj::real_t vx = attrib.vertices[3 * idx.vertex_index + 0];
				tinyobj::real_t vy = attrib.vertices[3 * idx.vertex_index + 1];
				tinyobj::real_t vz = attrib.vertices[3 * idx.vertex_index + 2];
				tinyobj::real_t nx = attrib.normals[3 * idx.normal_index + 0];
				tinyobj::real_t ny = attrib.normals[3 * idx.normal_index + 1];
				tinyobj::real_t nz = attrib.normals[3 * idx.normal_index + 2];
				//tinyobj::real_t tx = attrib.texcoords[2 * idx.texcoord_index + 0];
				//tinyobj::real_t ty = attrib.texcoords[2 * idx.texcoord_index + 1];
				// Optional: vertex colors
				 tinyobj::real_t red = attrib.colors[3*idx.vertex_index+0];
				 tinyobj::real_t green = attrib.colors[3*idx.vertex_index+1];
				 tinyobj::real_t blue = attrib.colors[3*idx.vertex_index+2];
				 
				 
				 // set vertex info
				 tVertex * cur_v = new tVertex();
				 cur_v->mVertexId = idx.vertex_index;
				 cur_v->mPos = tVector(vx, vy, vz, 1);
				 cur_v->mNormal = tVector(nx, ny, nz, 0);
				 obj_->AddVertex(cur_v);

				 // set face info
				 cur_face->mVertexIdList[v] = idx.vertex_index;
				 cur_face->mVertexPtrList[v] = cur_v;
			}
			index_offset += fv;

			// per-face material
			shapes[s].mesh.material_ids[f];
		}
	}
	obj_->BuildEdgeList();
	std::cout << "[log] load obj file " << name << " succ." << std::endl;
	obj_->PrintInfo();
	return obj_;
}