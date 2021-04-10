#pragma once
#include "json/json.h"
#include <string>

class cJsonUtil
{
public:
    static bool ParseJson(const std::string &path, Json::Value &value);
};
