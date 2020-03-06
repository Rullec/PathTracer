#pragma once
#include <string>
#include <geometry/cObjMesh.h>

struct tMeshParams {
	// basic setting for loading mesh.
	std::string name;
	eMeshType type;
	double scale;
	tVector displacement;
	bool build_edge_info;
	bool shape_analysis;
	tMeshParams();
};

class cMeshLoader
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	cMeshLoader();
	// std::shared_ptr<cBaseMesh> cMeshLoader::Load(const cMeshLoader::tParams & params_)
	static std::shared_ptr<cBaseMesh> Load(const tMeshParams & params_);

private:
	static std::shared_ptr<cObjMesh> LoadObj(const std::string & name, double scale, const tVector & disp, bool build_edge, bool shape_analysis);
};

