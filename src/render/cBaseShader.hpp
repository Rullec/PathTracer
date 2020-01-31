#pragma once
#include <iostream>
// #include <string>

class cBaseShader{
public:
    cBaseShader(std::string, unsigned int );
    ~cBaseShader();
    void ReadSrcFromFile(std::string src_path);
    unsigned int GetShaderHandle();

private:
    void CompileSrc();
    const unsigned int INVALID_HANDLE = INT16_MAX;
    std::string mSrcPath;
    std::string mSrc;
    unsigned int mShaderHandle;
    unsigned int mShaderType;
};