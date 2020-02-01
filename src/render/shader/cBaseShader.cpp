#include "cBaseShader.hpp"
#include <GL/glew.h>
// #include <GLFW/glfw3.h>
#include <iostream>
#include <fstream>
#include <string>

// -------------------BaseShader begin-----------------------
cBaseShader::cBaseShader(std::string src_path, unsigned int shader_type)
{
    if(GL_VERTEX_SHADER != shader_type && GL_FRAGMENT_SHADER != shader_type)
    {
        std::cout <<"[error] cBase Shader Unsupported shader type: " << shader_type << std::endl;
        exit(1);
    }

    // init member vars
    mSrcPath = src_path;
    mSrc.clear();
    mShaderHandle = 0;
    mShaderType = shader_type;

    // read src from file
    ReadSrcFromFile(mSrcPath);

    // compile the source code 
    CompileSrc();
}

cBaseShader::~cBaseShader()
{
    if(INVALID_HANDLE != mShaderHandle)
    {
        glDeleteShader(mShaderHandle);
    }
}

void cBaseShader::ReadSrcFromFile(std::string filename)
{
	std::ifstream ifile(filename);
	std::string filetext;

	while (ifile.good()) {
		std::string line;
		std::getline(ifile, line);
		filetext.append(line + "\n");
	}
	mSrc = filetext;
} 

unsigned int cBaseShader::GetShaderHandle()
{
    // get shader
    if(INVALID_HANDLE == mShaderHandle)
    {
        std::cout <<"[error] shader handle is invalid for " << mSrcPath << std::endl;
        exit(1);
    }
    return mShaderHandle;
}

void cBaseShader::CompileSrc()
{
    mShaderHandle = glCreateShader(mShaderType);
	const GLchar *source = (const GLchar *)mSrc.c_str();
    glShaderSource(mShaderHandle, 1, &source, 0);
    glCompileShader(mShaderHandle);

    // check the success of compliation
    int success, logsize;
    glGetShaderiv(mShaderHandle, GL_INFO_LOG_LENGTH, &logsize);
    char * infoLog = new char[logsize + 1];
    memset(infoLog, 0, sizeof(char) * (logsize + 1));
    glGetShaderiv(mShaderHandle, GL_COMPILE_STATUS, &success);

    if(GL_FALSE == success)
    {
        glGetShaderInfoLog(mShaderHandle, logsize + 1, NULL, infoLog);
        std::cout << "[error] Failed to compile shader " << mSrcPath << ": \n\t"<< infoLog << std::endl;
        exit(1);
    }
	else
	{
		std::cout << "[log] cBaseShader: load shader " << mSrcPath << " succ" << std::endl;
	}
}
// -------------------BaseShader end---------------------