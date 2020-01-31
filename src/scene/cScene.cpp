#include "cScene.hpp"
#include <iostream>

cScene::cScene()
{
    mSceneStatus = BeforeInit;
}

cScene::~cScene()
{
    // std::cout << "[log] base scene deconstructed" << std::endl;
}
