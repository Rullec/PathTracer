#pragma once
#include <string>
#include <geometry/cBaseMesh.h>

class cMeshLoader
{
public:
	cMeshLoader();
	static std::shared_ptr<cBaseMesh> Load(const std::string & name, eMeshType type);

private:
	static std::shared_ptr<cObjMesh> LoadObj(const std::string & name);
};

