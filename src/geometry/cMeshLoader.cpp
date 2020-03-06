#include "cMeshLoader.h"
#include <iostream>
#define TINYOBJLOADER_IMPLEMENTATION 
#include <tinyobj/tiny_obj_loader.h>
#include <stb_image.h>
#include <util/cFileUtil.h>
using namespace std;

tMeshParams::tMeshParams()
{
	name = "";
	type = eMeshType::OBJ;
	scale = 1.0;
	displacement = tVector::Zero();
	build_edge_info = false;
	shape_analysis = false;
}

cMeshLoader::cMeshLoader()
{

}

std::shared_ptr<cBaseMesh> cMeshLoader::Load(const tMeshParams & params_)
{
	std::shared_ptr<cBaseMesh> mesh_ = nullptr;
	switch (params_.type)
	{
	case OBJ:
	{
		// std::cout <<"-------------- load obj " << params_.name << "--------------\n";
		// std::cout <<"scale = " << params_.scale <<", displacement = " << params_.displacement.transpose() <<", build edge and shape = " << params_.build_edge_info <<" " << params_.shape_analysis << std::endl;
		std::shared_ptr<cObjMesh> x = LoadObj(params_.name, params_.scale, params_.displacement, params_.build_edge_info, params_.shape_analysis);
		mesh_ = std::dynamic_pointer_cast<cBaseMesh> (x);
	}
		break;
	default:
	{
		std::cout << "[error] cMeshLoader::Load: Unsupported mesh type " << params_.type << std::endl;
		exit(1);
	}
		break;
	}

	return mesh_;
}

std::shared_ptr<cObjMesh> cMeshLoader::LoadObj(const std::string & name, double scale, const tVector & disp, bool build_edge, bool shape_analysis)
{
	auto obj_ = (std::shared_ptr<cObjMesh>)(new cObjMesh(name));

	tinyobj::attrib_t attrib;
	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::material_t> materials;
	if (false == tinyobj::LoadObj(&attrib, &shapes, &materials, nullptr, nullptr, name.c_str(), cFileUtil::GetDirname(name).c_str())) //, "./model/cbox"
	{
		std::cout << "[error] cMeshLoader::LoadObj " << name << " failed " << std::endl;
		exit(1);
	}

	// std::cout <<"[debug] material size = " << materials.size() << std::endl; 
	auto Array3TotVector = [](float * ptr)
	{
		return tVector(ptr[0], ptr[1], ptr[2], 0);
	};
	for(auto & x : materials)
	{
		// std::cout <<"[debug] add material " << x.name << std::endl;
		tVector diffuse = Array3TotVector(x.diffuse),
			ambient = Array3TotVector(x.ambient),
			specular = Array3TotVector (x.specular),
			transmittance = Array3TotVector(x.transmittance);
		double ior = x.ior,
			shininess = x.shininess,
			illum = x.illum;

		tMaterial * material = new tMaterial();
		material->diffuse = diffuse;
		material->ambient = ambient;
		material->specular = specular;
		material->transmittance = transmittance;
		material->ior = ior;
		material->shininess = shininess;
		material->illum = illum;
		// std::cout <<"diffuse = " << diffuse.transpose() << std::endl;
		// std::cout <<"ambient = " << ambient.transpose() << std::endl;
		// std::cout <<"specular = " << specular.transpose() << std::endl;
		// std::cout <<"transmittance Tf = " << transmittance.transpose() << std::endl;
		// std::cout <<"ior Ni = " << ior << std::endl;
		// std::cout <<"shininess Ns = " << shininess << std::endl;
		// std::cout <<"illum = " << illum << std::endl;
		obj_->AddMaterial(material);
	}

	size_t face_id_before = 0;
	for (size_t shape_id = 0; shape_id < shapes.size(); shape_id++) 
	{
		// std::cout << "shape size = " << shapes.size() << std::endl;

		// Loop over faces(polygon)
		size_t index_offset = 0;
		for (size_t face_id = 0; face_id < shapes[shape_id].mesh.num_face_vertices.size(); face_id++) 
		{
			// if(face_id % 1000 == 0)
			// 	std::cout << "materil id = " << shapes[shape_id].mesh.material_ids[face_id] << std::endl;
			int fv = shapes[shape_id].mesh.num_face_vertices[face_id];
			if (NUM_VERTEX_PER_FACE != fv)
			{
				std::cout << "[error] cMeshLoader::LoadObj: didn't support faces with " << fv << " vertices" << std::endl;
				exit(1);
			}

			tFace *cur_face = new tFace();
			cur_face->mFaceId = face_id_before + face_id;
			cur_face->mMaterialId = shapes[shape_id].mesh.material_ids[face_id]; // make the material id count from 0
			cur_face->mShapeId = shape_id;
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

	if(build_edge) obj_->BuildEdgeList();
	// if(shape_analysis) obj_->ShapeAnalysis();
	std::cout << "[log] load obj file " << name << " succ." << std::endl;
	obj_->PrintInfo();
	return obj_;
}