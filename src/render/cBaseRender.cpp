#include "cBaseRender.hpp"
#include "cBaseShader.hpp"
#include <json/json.h>
#include <fstream>

cBaseRender::cBaseRender(const std::string & a_): mConfPath(a_)
{
	// load value
	std::ifstream fin(mConfPath);
	Json::Value root;
	Json::Reader reader;
	reader.parse(fin, root);

	Json::Value render = root["Render"];
	for (int i = 0; i < mClearColor.size(); i++) 
		mClearColor[i] = render["ClearColor"][i].asDouble();

	mVertexShaderPath = render["vertex_shader_path"].asString();
	mFragmentShaderPath = render["fragment_shader_path"].asString();

	mCamera = nullptr;
	// set up inits
	mPixelNum = 0;
	memset(mPixelBuffer, 0, sizeof(mPixelBuffer));
	mLineNum = 0;
	memset(mLineBuffer, 0, sizeof(mLineBuffer));
	mFaceNum = 0;
	memset(mFaceBuffer, 0, sizeof(mFaceBuffer));

	mPointsVAO = -1;
	mPointsVBO = -1;
	mLinesVAO = -1;
	mLinesVBO = -1;
	mFacesVAO = -1;
	mFacesVBO = -1;
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

	// add axis: clear work
	Clear();
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
	
	glUseProgram(mShaderProgram);

	glBindVertexArray(mPointsVAO);
	glDrawArrays(GL_POINTS, 0, mPixelNum);

	glBindVertexArray(mLinesVAO);
	glDrawArrays(GL_LINES, 0, mLineNum * 2);

	//std::cout << "draw array: face num = " << mFaceNum << std::endl;;
	glBindVertexArray(mFacesVAO);
	glDrawArrays(GL_TRIANGLES, 0, mFaceNum * 3);
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

void cBaseRender::SetCamera(std::shared_ptr<cFocusCamera>& camera)
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
		//mFaceBuffer[st + 3] = 0.7;
		//mFaceBuffer[st + 4] = 0.7;
		//mFaceBuffer[st + 5] = 0.7;
		//mFaceBuffer[st + 6] = 1;
	}
	mFaceNum++;
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
}

void cBaseRender::UpdateCamera()
{
	//std::cout << "[debug] cBaseRender::UpdateCamera \n";
	tMatrix res = mCamera->GetRenderMat();
	//res = tMatrix::Identity();
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
	Eigen::Matrix4f res = mat.cast<float>();
	glUniformMatrix4fv(pos, 1, GL_TRUE, res.data());
}