
#include "cJsonUtil.hpp"
#include <fstream>
#include <iostream>

bool cJsonUtil::ParseJson(const std::string &path, Json::Value &value)
{
    std::ifstream fin(path);
    if (fin.fail() == true)
    {
        std::cout << "[error] cJsonUtil::ParseJson file " << path
                  << " doesn't exist\n";
        return false;
    }
    Json::CharReaderBuilder rbuilder;
    std::string errs;
    bool parsingSuccessful =
        Json::parseFromStream(rbuilder, fin, &value, &errs);
    if (!parsingSuccessful)
    {
        // report to the user the failure and their locations in the document.
        std::cout
            << "[error] cJsonUtil::ParseJson: Failed to parse configuration\n"
            << errs << std::endl;
        return false;
    }
    return true;
}