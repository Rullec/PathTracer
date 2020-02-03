#pragma once
#include <string>
#include <geometry/cBaseMesh.h>

class cMeshLoader
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	cMeshLoader();
	static std::shared_ptr<cBaseMesh> Load(const std::string & name, eMeshType type, double scale = 1.0, const tVector & displacement = tVector::Zero());

private:
	static std::shared_ptr<cObjMesh> LoadObj(const std::string & name, double scale, const tVector & disp);
};

