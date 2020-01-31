#include "cBaseRender.hpp"
#include "cPolyRender.hpp"
#include <json/json.h>
#include <fstream>
#include <iostream>

void BuildRender(const std::string & conf, std::shared_ptr<cBaseRender> & render_)
{
	std::ifstream fin(conf.c_str());
	if (fin.fail() == true)
	{
		std::cout << "[error] BuildRender: load " << conf << " failed" << std::endl;
		exit(1);
	}
	Json::Reader reader;
	Json::Value root;
	if (false == reader.parse(fin, root))
	{
		std::cout << "[error] BuildRender: load json succ, failed to parse: " << conf << std::endl;
		exit(1);
	}
	fin.close();

	// render info
	Json::Value render_info = root["Render"];
	std::string render_type = render_info["Type"].asString();
	std::shared_ptr<cBaseRender> render = nullptr;;
	for (int i = 0; i < eRenderType::NUM_RENDER_TYPE && render == nullptr; i++)
	{
		if (render_type != gRenderName[i]) continue;
		
		// it is
		switch (i)
		{
		case eRenderType::POLY_RENDER: render = (std::shared_ptr<cBaseRender>)(new cPolyRender(conf)); break;
			default: break;
		}
	}
	if (nullptr == render)
	{
		std::cout << "[error] BuildRender: error render type " << render_type;
		exit(1);
	}
	
	render_ = render;
}