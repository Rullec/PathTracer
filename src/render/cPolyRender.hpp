#pragma once
#include "cBaseRender.hpp"

class cPolyRender : public cBaseRender {
public:
	cPolyRender(const std::string & conf);
	void Init() override final;

protected:

};