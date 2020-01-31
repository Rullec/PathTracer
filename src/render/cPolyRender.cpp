#include "cPolyRender.hpp"

cPolyRender::cPolyRender(const std::string &conf):cBaseRender(conf)
{

}

void cPolyRender::Init()
{
	cBaseRender::Init();

	this->mRenderStatus = eRenderStatus::INIT_SUCC;
}