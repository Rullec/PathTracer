#include "cMeshLoader.h"
#include <iostream>
#define TINYOBJLOADER_IMPLEMENTATION 
#include <tinyobj/tiny_obj_loader.h>
#include <stb_image.h>
#include <util/cFileUtil.h>
using namespace std;

cMeshLoader::cMeshLoader()
{

}

std::shared_ptr<cBaseMesh> cMeshLoader::Load(const std::string & name, eMeshType type, double scale, const tVector &displacement)
{
	std::shared_ptr<cBaseMesh> mesh_ = nullptr;
	switch (type)
	{
	case OBJ:
	{
		std::shared_ptr<cObjMesh> x = LoadObj(name, scale, displacement);
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

std::shared_ptr<cObjMesh> cMeshLoader::LoadObj(const std::string & name, double scale, const tVector & disp)
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

	size_t face_id_before = 0;
	for (size_t shape_id = 0; shape_id < shapes.size(); shape_id++) 
	{
		// std::cout << "shape size = " << shapes.size() << std::endl;

		// Loop over faces(polygon)
		size_t index_offset = 0;
		for (size_t face_id = 0; face_id < shapes[shape_id].mesh.num_face_vertices.size(); face_id++) 
		{
			int fv = shapes[shape_id].mesh.num_face_vertices[face_id];
			if (NUM_VERTEX_PER_FACE != fv)
			{
				std::cout << "[error] cMeshLoader::LoadObj: didn't support faces with " << fv << " vertices" << std::endl;
				exit(1);
			}

			tFace *cur_face = new tFace();
			cur_face->mFaceId = face_id_before + face_id;
			obj_->AddFace(cur_face);

			// Loop over vertices in the face.
			for (size_t v = 0; v < fv; v++) {
				// access to vertex
				tVertex * cur_v = new tVertex();
				
				tinyobj::index_t idx = shapes[shape_id].mesh.indices[index_offset + v];
				//std::cout << "vertex = " << idx.vertex_index << std::endl;
				tinyobj::real_t vx = (attrib.vertices[3 * idx.vertex_index + 0]+ disp[0]) * scale  ;
				tinyobj::real_t vy = (attrib.vertices[3 * idx.vertex_index + 1] + disp[1])* scale;
				tinyobj::real_t vz = (attrib.vertices[3 * idx.vertex_index + 2] + disp[2])* scale;


				// load vertices normal 
				tinyobj::real_t nx = std::nan(""), ny = std::nan(""), nz = std::nan("");
				if(attrib.normals.size() > 0)
				{
					// std::cout <<"[debug] MeshLoader:: normal info available\n";
					
					nx = attrib.normals[3 * idx.normal_index + 0];
					ny = attrib.normals[3 * idx.normal_index + 1];
					nz = attrib.normals[3 * idx.normal_index + 2];
				}
				//tinyobj::real_t tx = attrib.texcoords[2 * idx.texcoord_index + 0];
				//tinyobj::real_t ty = attrib.texcoords[2 * idx.texcoord_index + 1];

				// Optional: vertex colors
				tinyobj::real_t red = attrib.colors[3*idx.vertex_index+0];
				tinyobj::real_t green = attrib.colors[3*idx.vertex_index+1];
				tinyobj::real_t blue = attrib.colors[3*idx.vertex_index+2];

				//  std::cout <<"color = " << red << green << blue << std::endl;
				tinyobj::real_t texCoord_x, texCoord_y;
				if(attrib.texcoords.size() > 0)
				{
					texCoord_x = attrib.texcoords[2 * idx.vertex_index + 0];
					texCoord_y = attrib.texcoords[2 * idx.vertex_index + 1];
				}
				
				// set vertex info
				// std::cout << idx.vertex_index << std::endl;
				cur_v->mVertexId = idx.vertex_index;
				cur_v->mPos = tVector(vx, vy, vz, 1);
				cur_v->mNormal = tVector(nx, ny, nz, 0);
				cur_v->mColor = tVector(red, green, blue, 1);
				cur_v->mTexCoord = Eigen::Vector2d(texCoord_x, texCoord_y);
				// std::cout <<"[debug] tex coordinate = " << cur_v->mTexCoord.transpose() << std::endl;

				
				// std::cout <<"[debug] mesh loader, for pt " << cur_v->mPos.transpose() <<", tex cor = " << cur_v->mTexCoord.transpose() << std::endl;
				//  std::cout << cur_v->mColor.transpose() << std::endl;
				obj_->AddVertex(cur_v);

				// exand the upper & lower boud for this obj
				
				// set face info
				cur_face->mVertexIdList[v] = idx.vertex_index;
				cur_face->mVertexPtrList[v] = cur_v;
			}
			index_offset += fv;

			// per-face material
			shapes[shape_id].mesh.material_ids[face_id];
		}

		face_id_before += shapes[shape_id].mesh.num_face_vertices.size();
	}
	
	// detect and load texture file by stb_image
	{
		std::string texture_root = cFileUtil::RemoveExtension(name);
		// std::cout <<"[debug] texture root = " << texture_root;
		// detect jpg and png image file for this name
		std::string texture_file = "";
		if(true == cFileUtil::ExistsFile(texture_root + ".jpg"))
		{
			texture_file = texture_root + ".jpg";
		}
		else if (true == cFileUtil::ExistsFile(texture_root + ".png"))
		{
			texture_file = texture_root + ".png";
		}
		// std::cout <<"[debug] get texture file = " << texture_file << std::endl;
		if(texture_file.size() !=0 )
		{
			int width, height, nrChannels;
			// stbi_set_flip_vertically_on_load(true); // tell stb_image.h to flip loaded texture's on the y-axis.

			// The FileSystem::getPath(...) is part of the GitHub repository so we can find files on any IDE/platform; replace it with your own image path.
			unsigned char *data = stbi_load(texture_file.c_str(), &width, &height, &nrChannels, 0);
			// assert(nrChannels == 3);
			obj_->SetTexture(data, width, height);
			// std::cout <<" channels = " << nrChannels << std::endl;
			// exit(1);
		}
	}

	obj_->BuildEdgeList();
	std::cout << "[log] load obj file " << name << " succ." << std::endl;
	obj_->PrintInfo();
	return obj_;
}