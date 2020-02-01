#pragma once
#include <string>
#include <json/json.h>

class cJsonUtil{
public:
    static bool ParseJson(const std::string & path, Json::Value & value);
};
