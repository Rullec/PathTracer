#pragma once
#include <string>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <util/cMathUtil.hpp>
#include <geometry/cBaseMesh.h>
#include <render/cFocusCamera.h>

#define MAX_PIXEL_NUM (1920 * 1080)
#define MAX_LINE_NUM (1000000)
#define MAX_FACE_NUM (100000)

enum eRenderType {
	POLY_RENDER=0,
	NUM_RENDER_TYPE,
};

const std::string gRenderName[] = {
	"Poly Render"
};

class cBaseRender {
public:
	enum eRenderStatus {
		NOT_INIT = 0,
		INIT_SUCC,
	};

	cBaseRender(const std::string &str_);
	virtual ~cBaseRender() = 0;

	virtual void Init();
	virtual void Draw();
	virtual void Clear();

	void AddPixel(const tPixel & pixs);
	void AddLine(const tEdge & line);
	void AddLine(const tLine & line);
	void AddFace(tVertex ** tPixel_lst);
	void AddPolygon(const tPolygon & face);

	void SetCamera(std::shared_ptr<cFocusCamera> & camera);

protected:

	// pipeline essentials
	const std::string mConfPath;
	tVector mClearColor;
	std::string mVertexShaderPath, mFragmentShaderPath;
	unsigned int mShaderProgram;
	enum eRenderStatus mRenderStatus;
	std::shared_ptr<cFocusCamera> mCamera;

	GLuint mPointsVAO, mPointsVBO;
	GLuint mLinesVAO, mLinesVBO;
	GLuint mFacesVAO, mFacesVBO;

	bool mNeedReload;

	// buffers
	int mPixelNum;
	float mPixelBuffer[MAX_PIXEL_NUM * tPixel::size];
	int mLineNum;
	float mLineBuffer[MAX_LINE_NUM * tEdge::size];
	int mFaceNum;
	float mFaceBuffer[MAX_FACE_NUM * tFace::size];

	// pipeline methods
	void InitShader();
	void Reload();
	void UpdateCamera();

	// set uniform values
	void SetBool(const std::string & name, bool value) const;
	void SetVector(const std::string, const tVector & vector) const;
	void SetMatrix(const std::string, const tMatrix & mat) const;
};