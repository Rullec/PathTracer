#include "cScene.hpp"
#include "cDrawScene.hpp"

std::shared_ptr<cScene> BuildScene(std::string path)
{
	std::shared_ptr<cScene> scene = (std::shared_ptr<cScene>)(new cDrawScene(path));
	return scene;
}