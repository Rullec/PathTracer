#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "cBaseRender.hpp"
#include "./shader/cBaseShader.hpp"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include <util/cJsonUtil.hpp>
#include <geometry/cMeshLoader.h>
#include <fstream>

cBaseRender::cBaseRender(const std::string & a_): mConfPath(a_)
{
	// load value
	Json::Value root;
	cJsonUtil::ParseJson(mConfPath, root);

	Json::Value render = root["Render"];
	for (int i = 0; i < mClearColor.size(); i++) 
		mClearColor[i] = render["ClearColor"][i].asDouble();

	mVertexShaderPath = render["vertex_shader_path"].asString();
	mFragmentShaderPath = render["fragment_shader_path"].asString();
	mGroundPath = render["ground_path"].asString();
	mGroundScale = render["ground_scale"].asDouble();
	for(int i = 0; i < 3; i++)
		mGroundMove[i] = render["ground_displacement"][i].asDouble();

	mCamera = nullptr;
	// set up inits
	mPixelNum = 0;
	memset(mPixelBuffer, 0, sizeof(mPixelBuffer));
	mLineNum = 0;
	memset(mLineBuffer, 0, sizeof(mLineBuffer));
	mFaceNum = 0;
	memset(mFaceBuffer, 0, sizeof(mFaceBuffer));
	mTexFaceNum = 0;
	memset(mTexFaceBuffer, 0, sizeof(mTexFaceBuffer));
	mTexInfo.clear();

	mPointsVAO = -1;
	mPointsVBO = -1;
	mLinesVAO = -1;
	mLinesVBO = -1;
	mFacesVAO = -1;
	mFacesVBO = -1;
	mTexFacesVAO = -1;
	mTexFacesVBO = -1;
	// mGroundVAO = -1;
	// mGroundVBO = -1;

	mNeedReload = true;

	mRenderStatus = eRenderStatus::NOT_INIT;
}

cBaseRender::~cBaseRender()
{

}

void cBaseRender::Init()
{
	// set clear color
	glClearColor(mClearColor[0], mClearColor[1], mClearColor[2], mClearColor[3]);

	// init shader
	InitShader();

	// init VAO, VBO
	glGenVertexArrays(1, &mPointsVAO);
	glGenBuffers(1, &mPointsVBO);
	glGenVertexArrays(1, &mLinesVAO);
	glGenBuffers(1, &mLinesVBO);
	glGenVertexArrays(1, &mFacesVAO);
	glGenBuffers(1, &mFacesVBO);
	glGenVertexArrays(1, &mTexFacesVAO);
	glGenBuffers(1, &mTexFacesVBO);

	// add axis: clear work
	Clear();

	// add text code & ground
	// AddTestCubeTex();
	InitGround();
}

void cBaseRender::InitShader()
{
	std::unique_ptr<cBaseShader> vertex_shader = (std::unique_ptr<cBaseShader>)(new cBaseShader(mVertexShaderPath, GL_VERTEX_SHADER)),
		fragment_shader = (std::unique_ptr<cBaseShader>)(new cBaseShader(mFragmentShaderPath, GL_FRAGMENT_SHADER));

	// create shader program after shaders
	int success = 1, logsize = 0;
	char * infoLog = nullptr;
	mShaderProgram = glCreateProgram();
	glAttachShader(mShaderProgram, vertex_shader->GetShaderHandle());
	glAttachShader(mShaderProgram, fragment_shader->GetShaderHandle());
	// std::cout <<"[cBaseRender] vertex shader handle = " << vertex_shader->GetShaderHandle() << std::endl;
	// std::cout <<"[cBaseRender] fragment shader handle = " << fragment_shader->GetShaderHandle() << std::endl;
	glLinkProgram(mShaderProgram);
	glGetProgramiv(mShaderProgram, GL_INFO_LOG_LENGTH, &logsize);
	infoLog = new char[logsize + 1];
	memset(infoLog, 0, sizeof(char) * (logsize + 1));
	glGetProgramiv(mShaderProgram, GL_LINK_STATUS, &success);
	if (GL_FALSE == success)
	{
		glGetProgramInfoLog(mShaderProgram, logsize + 1, NULL, infoLog);
		std::cout << "[cBaseRender] Shaders Link Error: \n" << infoLog << std::endl;
		exit(1);
	}

	// delete shaders after linking
	vertex_shader.reset();
	fragment_shader.reset();
}

void cBaseRender::Draw()
{
	if (mRenderStatus != eRenderStatus::INIT_SUCC)
	{
		std::cout << "[error] cBaseRender::Draw: render hasn't been initialized: " << mRenderStatus << std::endl;
		exit(1);
	}
	if (this->mCamera == nullptr)
	{
		std::cout << "[error] cBaseRender::Draw: no camera setting, error\n";
		exit(1);
	}
	
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	// update camera matrix
	{
		UpdateCamera();
	}

	// if data changed, reload data
	{
		if (mNeedReload == true) Reload();
	}
	
	// non texture objs: faces, lines, points.
	{
		glUseProgram(mShaderProgram);
		SetBool("gEnableTexture", false);

		glBindVertexArray(mPointsVAO);
		glDrawArrays(GL_POINTS, 0, mPixelNum);

		glBindVertexArray(mLinesVAO);
		glDrawArrays(GL_LINES, 0, mLineNum * 2);

		//std::cout << "draw array: face num = " << mFaceNum << std::endl;;
		glBindVertexArray(mFacesVAO);
		glDrawArrays(GL_TRIANGLES, 0, mFaceNum * 3);
	}

	// draw texture objs
	{
		SetBool("gEnableTexture", true);
		glUseProgram(mShaderProgram);
		glBindVertexArray(mTexFacesVAO);

		// 分obj绘制
		for(auto & tex_info : mTexInfo)
		{
			int st = tex_info.mOffsetSt,
				ed = tex_info.mOffsetEd;
			// std::cout <<"[debug] st, ed = " << st <<" " << ed << ", bind = " << tex_info.mTexBind << std::endl;
			glBindTexture(GL_TEXTURE_2D, tex_info.mTexBind);
			glDrawArrays(GL_TRIANGLES, NUM_VERTEX_PER_FACE * st, (ed - st) * NUM_VERTEX_PER_FACE);
		}

		// auto & tex_info = mTexInfo[0];
		// {
		// 	int st = tex_info.mOffsetSt,
		// 		ed = tex_info.mOffsetEd;
		// 	std::cout <<"[debug] st, ed = " << st <<" " << ed << ", bind = " << tex_info.mTexBind << std::endl;
		// 	glBindTexture(GL_TEXTURE_2D, tex_info.mTexBind);
		// 	glDrawArrays(GL_TRIANGLES, 0, 1 * NUM_VERTEX_PER_FACE);
		// 	glDrawArrays(GL_TRIANGLES, 3, 1 * NUM_VERTEX_PER_FACE);
		// }

		// glDrawArrays(GL_TRIANGLES, 0, mTexFaceNum * NUM_VERTEX_PER_FACE);
	}
}

void cBaseRender::AddPixel(const tPixel & pix)
{
	if (mPixelNum >= MAX_PIXEL_NUM)
	{
		std::cout << "[error] cPixelRender::AddPixel: exceed max pixel num" << MAX_PIXEL_NUM << std::endl;
		exit(1);
	}

	mNeedReload = true;
	int st = (mPixelNum++) * tPixel::size;
	mPixelBuffer[st + 0] = static_cast<float>(pix.mX);
	mPixelBuffer[st + 1] = static_cast<float>(pix.mY);
	mPixelBuffer[st + 2] = static_cast<float>(pix.mColor[0]);
	mPixelBuffer[st + 3] = static_cast<float>(pix.mColor[1]);
	mPixelBuffer[st + 4] = static_cast<float>(pix.mColor[2]);
	mPixelBuffer[st + 5] = static_cast<float>(pix.mColor[3]);
}

void cBaseRender::AddLine(const tLine & line)
{
	if (mLineNum >= MAX_LINE_NUM)
	{
		std::cout << "[error] cPixelRender::AddLine: exceed max line num" << MAX_LINE_NUM << std::endl;
		exit(1);
	}
	//std::cout << "line color = " << line.mColor << std::endl;
	//std::cout << "line num = " << mLineNum << std::endl;
	mNeedReload = true;
	int st = (mLineNum++) * tEdge::size;
	mLineBuffer[st + 0] = static_cast<float>(line.mOri[0]);
	mLineBuffer[st + 1] = static_cast<float>(line.mOri[1]);
	mLineBuffer[st + 2] = static_cast<float>(line.mOri[2]);
	mLineBuffer[st + 3] = static_cast<float>(line.mColor[0]);
	mLineBuffer[st + 4] = static_cast<float>(line.mColor[1]);
	mLineBuffer[st + 5] = static_cast<float>(line.mColor[2]);
	mLineBuffer[st + 6] = static_cast<float>(line.mColor[3]);
	st = st + tEdge::size / 2;
	mLineBuffer[st + 0] = static_cast<float>(line.mDest[0]);
	mLineBuffer[st + 1] = static_cast<float>(line.mDest[1]);
	mLineBuffer[st + 2] = static_cast<float>(line.mDest[2]);
	mLineBuffer[st + 3] = static_cast<float>(line.mColor[0]);
	mLineBuffer[st + 4] = static_cast<float>(line.mColor[1]);
	mLineBuffer[st + 5] = static_cast<float>(line.mColor[2]);
	mLineBuffer[st + 6] = static_cast<float>(line.mColor[3]);
}

void cBaseRender::SetCamera(std::shared_ptr<cBaseCamera>& camera)
{
	mCamera = camera;
}

void cBaseRender::AddLine(const tEdge & line)
{
	if (mLineNum >= MAX_LINE_NUM)
	{
		std::cout << "[error] cPixelRender::AddLine: exceed max line num" << MAX_PIXEL_NUM << std::endl;
		exit(1);
	}

	mNeedReload = true;
	int st = (mLineNum++) * tEdge::size;
	mLineBuffer[st + 0] = static_cast<float>(line.mOri->mPos[0]);
	mLineBuffer[st + 1] = static_cast<float>(line.mOri->mPos[1]);
	mLineBuffer[st + 2] = static_cast<float>(line.mOri->mPos[2]);
	mLineBuffer[st + 3] = static_cast<float>(line.mOri->mColor[0]);
	mLineBuffer[st + 4] = static_cast<float>(line.mOri->mColor[1]);
	mLineBuffer[st + 5] = static_cast<float>(line.mOri->mColor[2]);
	mLineBuffer[st + 6] = static_cast<float>(line.mOri->mColor[3]);
	st = st + tEdge::size / 2;
	mLineBuffer[st + 0] = static_cast<float>(line.mDest->mPos[0]);
	mLineBuffer[st + 1] = static_cast<float>(line.mDest->mPos[1]);
	mLineBuffer[st + 2] = static_cast<float>(line.mDest->mPos[2]);
	mLineBuffer[st + 3] = static_cast<float>(line.mDest->mColor[0]);
	mLineBuffer[st + 4] = static_cast<float>(line.mDest->mColor[1]);
	mLineBuffer[st + 5] = static_cast<float>(line.mDest->mColor[2]);
	mLineBuffer[st + 6] = static_cast<float>(line.mDest->mColor[3]);
}

void cBaseRender::AddFace(tVertex ** tVertex_lst)
{
	if (mFaceNum >= MAX_FACE_NUM)
	{
		std::cout << "[error] cPixelRender::AddFace: exceed max face num " << MAX_FACE_NUM << std::endl;
		exit(1);
	}

	mNeedReload = true;
	//std::cout << "add face: face num = " << mFaceNum << std::endl;;
	for (int i = 0; i < NUM_VERTEX_PER_FACE; i++)
	{
		const tVertex * cur_vertex = tVertex_lst[i];
		int st = mFaceNum * tFace::size + i * tVertex::size;
		//std::cout << "vertex " << i << " = " << cur_pixel.mX << " " << cur_pixel.mY << std::endl;
		mFaceBuffer[st + 0] = cur_vertex->mPos[0];
		mFaceBuffer[st + 1] = cur_vertex->mPos[1];
		mFaceBuffer[st + 2] = cur_vertex->mPos[2];
		//std::cout << cur_vertex->mPos.transpose() << std::endl;

		mFaceBuffer[st + 3] = cur_vertex->mColor[0];
		mFaceBuffer[st + 4] = cur_vertex->mColor[1];
		mFaceBuffer[st + 5] = cur_vertex->mColor[2];
		mFaceBuffer[st + 6] = cur_vertex->mColor[3];

		// add tex coordinate
		mFaceBuffer[st + 7]  = cur_vertex->mTexCoord[0];
		mFaceBuffer[st + 8]  = cur_vertex->mTexCoord[1];
	}
	mFaceNum++;
}

void cBaseRender::AddTexFace(tVertex ** tVertex_lst)
{
	if (mTexFaceNum >= MAX_FACE_NUM)
	{
		std::cout << "[error] cPixelRender::AddTexFace: exceed max face num " << MAX_FACE_NUM << std::endl;
		exit(1);
	}

	mNeedReload = true;
	//std::cout << "add face: face num = " << mFaceNum << std::endl;;
	for (int i = 0; i < NUM_VERTEX_PER_FACE; i++)
	{
		const tVertex * cur_vertex = tVertex_lst[i];
		int st = mTexFaceNum * tFace::size + i * tVertex::size;
		//std::cout << "vertex " << i << " = " << cur_pixel.mX << " " << cur_pixel.mY << std::endl;
		mTexFaceBuffer[st + 0] = cur_vertex->mPos[0];
		mTexFaceBuffer[st + 1] = cur_vertex->mPos[1];
		mTexFaceBuffer[st + 2] = cur_vertex->mPos[2];
		//std::cout << cur_vertex->mPos.transpose() << std::endl;

		mTexFaceBuffer[st + 3] = cur_vertex->mColor[0];
		mTexFaceBuffer[st + 4] = cur_vertex->mColor[1];
		mTexFaceBuffer[st + 5] = cur_vertex->mColor[2];
		mTexFaceBuffer[st + 6] = cur_vertex->mColor[3];

		// add tex coordinate
		mTexFaceBuffer[st + 7]  = cur_vertex->mTexCoord[0];
		mTexFaceBuffer[st + 8]  = cur_vertex->mTexCoord[1];
	}
	mTexFaceNum++;
}

void cBaseRender::AddPolygon(const tPolygon & face)
{
	mNeedReload = true;
	int v_num = face.mVertexLst.size();
	int face_num = v_num - 2;
	int v_id[NUM_VERTEX_PER_FACE];
	assert(NUM_VERTEX_PER_FACE == 3);

	for (int i = 0; i < face_num; i++)
	{
		v_id[0] = 0;
		v_id[1] = i + 1;
		v_id[2] = i + 2;
		for (int j = 0; j < NUM_VERTEX_PER_FACE; j++)
		{
			int st = mFaceNum * tFace::size + j * tVertex::size;
			//std::cout << "vertex " << i << " = " << cur_pixel.mX << " " << cur_pixel.mY << std::endl;
			mFaceBuffer[st + 0] = face.mVertexLst[v_id[j]][0];
			mFaceBuffer[st + 1] = face.mVertexLst[v_id[j]][1];
			mFaceBuffer[st + 2] = face.mVertexLst[v_id[j]][2];

			
			mFaceBuffer[st + 3] = 0.7;
			mFaceBuffer[st + 4] = 0.2;
			mFaceBuffer[st + 5] = 0.7;
			mFaceBuffer[st + 6] = 1;
		}
		mFaceNum++;
	}
}

void cBaseRender::Reload()
{
	// std::cout << std::rand() << "reload" << std::endl;
	// reload points
	{
		glBindVertexArray(mPointsVAO);
		glBindBuffer(GL_ARRAY_BUFFER, mPointsVBO);
		glBufferData(GL_ARRAY_BUFFER, mPixelNum * tPixel::size * sizeof(float), mPixelBuffer, GL_STATIC_DRAW);
		glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, tPixel::size * sizeof(float), (void *)0);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, tPixel::size * sizeof(float), (void *)(2 * sizeof(float)));
		glEnableVertexAttribArray(1);
	}


	// reload lines
	{
		glBindVertexArray(mLinesVAO);
		glBindBuffer(GL_ARRAY_BUFFER, mLinesVBO);
		glBufferData(GL_ARRAY_BUFFER, mLineNum * tEdge::size * sizeof(float), mLineBuffer, GL_STATIC_DRAW);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, (tEdge::size / 2) * sizeof(float), (void *)0);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, (tEdge::size / 2) * sizeof(float), (void *)(3 * sizeof(float)));
		glEnableVertexAttribArray(1);
	}

	// reload faces
	{
		glBindVertexArray(mFacesVAO);
		glBindBuffer(GL_ARRAY_BUFFER, mFacesVBO);
		glBufferData(GL_ARRAY_BUFFER, mFaceNum * tFace::size * sizeof(float), mFaceBuffer, GL_STATIC_DRAW);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, tVertex::size * sizeof(float), (void *)0);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, tVertex::size * sizeof(float), (void *)(3 * sizeof(float)));
		glEnableVertexAttribArray(1);
	}

	{
		glBindVertexArray(mTexFacesVAO);
		glBindBuffer(GL_ARRAY_BUFFER, mTexFacesVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * tFace::size * mTexFaceNum, mTexFaceBuffer, GL_STATIC_DRAW);

		// position attribute
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(0);
		// color attribute
		glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)(3 * sizeof(float)));
		glEnableVertexAttribArray(1);
		// texture coord attribute
		glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)(7 * sizeof(float)));
		glEnableVertexAttribArray(2);

		// set up texture
		for(auto & tex_info :mTexInfo)
		{
			// std::cout << mTexInfo[0].mTexBind << std::endl;
			if(tex_info.mTexBind != -1) continue;
				
			glGenTextures(1, &tex_info.mTexBind);
			glBindTexture(GL_TEXTURE_2D, tex_info.mTexBind); 
			// set the texture wrapping parameters
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);	// set texture wrapping to GL_REPEAT (default wrapping method)
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
			// set texture filtering parameters
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			// load image, create texture and generate mipmaps

			if (tex_info.mTexPtr)
			{
				glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, tex_info.mTexWidth, tex_info.mTexHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, tex_info.mTexPtr );
				glGenerateMipmap(GL_TEXTURE_2D);
			}
			else
			{
				std::cout << "Failed to load texture" << std::endl;
			}
		}
		
	}

	mNeedReload = false;
}

void cBaseRender::UpdateCamera()
{
	//std::cout << "[debug] cBaseRender::UpdateCamera \n";
	tMatrix res = mCamera->GetRenderMat();
	// std::cout <<"res = " << res << std::endl;
	SetMatrix("MVP", res);
}

void cBaseRender::Clear()
{
	mNeedReload = true;
	mPixelNum = 0;
	mLineNum = 0;
	mFaceNum = 0;
}

void cBaseRender::SetBool(const std::string & name, bool value) const
{
	glUniform1i(glGetUniformLocation(mShaderProgram, name.c_str()), (int)value);
}

void cBaseRender::SetVector(const std::string name, const tVector & vector) const
{
	glUniform4d(glGetUniformLocation(mShaderProgram, name.c_str()), vector[0], vector[1], vector[2], vector[3]);
}

void cBaseRender::SetMatrix(const std::string name, const tMatrix & mat) const
{
	GLint pos = glGetUniformLocation(mShaderProgram, name.c_str());
	if (pos == GL_INVALID_VALUE || pos == GL_INVALID_OPERATION)
	{
		std::cout << "[error] cBaseRender::SetMatrix failed" << std::endl;
		exit(1);
	}
	Eigen::Matrix4f res = mat.transpose().cast<float>();
	glUniformMatrix4fv(pos, 1, GL_TRUE, res.data());
}

void cBaseRender::AddTestCubeTex()
{
	float vertices[] = {
        // positions          // colors           // texture coords
         0.5f,  0.5f, 0.0f,   1.0f, 0.0f, 0.0f, 1.0f,   1.0f, 1.0f, // top right
         0.5f, -0.5f, 0.0f,   0.0f, 1.0f, 0.0f, 1.0f,   1.0f, 0.0f, // bottom right
        -0.5f,  0.5f, 0.0f,   1.0f, 1.0f, 0.0f, 1.0f,   0.0f, 1.0f,  // top left 

		0.5f, -0.5f, 0.0f,   0.0f, 1.0f, 0.0f, 1.0f,   1.0f, 0.0f, // bottom right
        -0.5f, -0.5f, 0.0f,   0.0f, 0.0f, 1.0f, 1.0f,   0.0f, 0.0f, // bottom left
        -0.5f,  0.5f, 0.0f,   1.0f, 1.0f, 0.0f, 1.0f,   0.0f, 1.0f  // top left 
    };

	// 直接写入buffer
	tTexInfo tex_info;
	tex_info.mOffsetSt = mTexFaceNum;
	memcpy(mTexFaceBuffer, vertices, sizeof(vertices));
	mTexFaceNum +=2;
	tex_info.mOffsetEd = mTexFaceNum;
	
	// exit(1);
    // unsigned int mGroundVBO, mGroundVAO, mGroundEBO;
    // glGenVertexArrays(1, &mTexFacesVAO);
    // glGenBuffers(1, &mTexFacesVBO);

    stbi_set_flip_vertically_on_load(true); // tell stb_image.h to flip loaded texture's on the y-axis.
    // The FileSystem::getPath(...) is part of the GitHub repository so we can find files on any IDE/platform; replace it with your own image path.
    tex_info.mTexPtr = stbi_load("ground/ground.png", &tex_info.mTexWidth, &tex_info.mTexHeight, &tex_info.mChannels, 0);
	mTexInfo.push_back(tex_info);
}

void cBaseRender::InitGround()
{
	// std::cout <<"[debug] init ground "; 
	// exit(1);

	// InitGround
	// load data
	std::shared_ptr<cBaseMesh> mesh = cMeshLoader::Load(mGroundPath, eMeshType::OBJ, mGroundScale, mGroundMove);

	AddMesh(mesh);
}

void cBaseRender::AddMesh(std::shared_ptr<cBaseMesh> &mesh)
{
	if(eMeshType::NUM_MESH_TYPE != 1)
	{
		std::cout <<"[error] cBaseRender::AddMesh: this method need redesigned!\n";
		exit(1);
	}
	eMeshType type = mesh->GetType();
	if(type == eMeshType::OBJ)
	{
		// draw method differs between texture mesh & non-texture mesh
		std::shared_ptr<cObjMesh> obj_mesh= std::dynamic_pointer_cast<cObjMesh>(mesh);
		if(nullptr == obj_mesh)
		{
			std::cout <<"[error] cBaseRender::AddMesh: obj mesh failed\n";
			exit(1);
		}

		// get texture and try to paint
		unsigned char * texture;
		int tex_width, tex_height;
		obj_mesh->GetTexture(texture, tex_width, tex_height);
		
		// std::cout <<"[debug] cBaseRender::getTex = " << (nullptr == texture) << std::endl;
		if(texture == nullptr)
		{
			// std::cout <<"[log] cBaseRender::AddMesh obj mesh has no texture\n";
			std::vector<tFace * > face_lst = mesh->GetFaceList();
			for (auto & x : face_lst)
				AddFace(x->mVertexPtrList);
		}
		else
		{
			// std::cout <<"[log] cBaseRender::AddMesh obj mesh has a texture\n";

			// set up text_info & generate
			cBaseRender::tTexInfo tex_info;
			tex_info.mOffsetSt = mTexFaceNum;
			std::vector<tFace * > face_lst = mesh->GetFaceList();
			for (auto & x : face_lst)
				AddTexFace(x->mVertexPtrList);
			
			tex_info.mOffsetEd = mTexFaceNum;
			obj_mesh->GetTexture(tex_info.mTexPtr, tex_info.mTexWidth, tex_info.mTexHeight);
			
			// create texture
			glGenTextures(1, &tex_info.mTexBind);
			glBindTexture(GL_TEXTURE_2D, tex_info.mTexBind); 
			// set the texture wrapping parameters
			// glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);	// set texture wrapping to GL_REPEAT (default wrapping method)
			// glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
			// set texture filtering parameters
			// r(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			// glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			// load image, create texture and generate mipmaps

			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, tex_info.mTexWidth, tex_info.mTexHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, tex_info.mTexPtr);
			glGenerateMipmap(GL_TEXTURE_2D);
			
			// push in
			mTexInfo.push_back(tex_info);
			// std::cout <<" create for texture " << mTexInfo.size() << std::endl;
		}
	}
	mNeedReload = true;
}