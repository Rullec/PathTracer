#pragma once
#include <string>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <util/cMathUtil.hpp>
#include <geometry/cBaseMesh.h>
#include "camera/cArcballCamera.hpp"

#define MAX_PIXEL_NUM (1920 * 1080)
#define MAX_LINE_NUM (100000)
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
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
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
	void AddTexFace(tVertex ** tVertex_lst);
	void AddPolygon(const tPolygon & face);

	void AddMesh(std::shared_ptr<cBaseMesh> &mesh);

	void SetCamera(std::shared_ptr<cBaseCamera> & camera);
	void InitGround();

protected:

	// pipeline essentials
	const std::string mConfPath;
	tVector mClearColor;
	std::string mVertexShaderPath, mFragmentShaderPath;
	std::string mGroundPath;
	double mGroundScale;
	tVector mGroundMove;
	unsigned int mShaderProgram;
	enum eRenderStatus mRenderStatus;
	std::shared_ptr<cBaseCamera> mCamera;

	GLuint mPointsVAO, mPointsVBO;
	GLuint mLinesVAO, mLinesVBO;
	GLuint mFacesVAO, mFacesVBO;
	GLuint mTexFacesVAO, mTexFacesVBO;
	// GLuint mGroundVAO, mGroundVBO, mGroundTex;

	bool mNeedReload;

	// buffers for non-texture objs
	int mPixelNum;
	float mPixelBuffer[MAX_PIXEL_NUM * tPixel::size];
	int mLineNum;
	float mLineBuffer[MAX_LINE_NUM * tEdge::size];
	int mFaceNum;
	float mFaceBuffer[MAX_FACE_NUM * tFace::size];

	// buffers for texture objs
	int mTexFaceNum;
	float mTexFaceBuffer[MAX_FACE_NUM * tFace::size];

	// each obj have its own tex, so we need a structure to remember it.
	struct tTexInfo{
		int mOffsetSt, mOffsetEd;	// offset in mTexFaceBuffer
		int mTexWidth, mTexHeight, mChannels;
		unsigned char * mTexPtr;
		unsigned int mTexBind;
		tTexInfo()
		{
			mOffsetSt = mOffsetEd = -1;
			mTexWidth = mTexHeight = -1;
			mTexPtr = nullptr;
			mTexBind = -1;
		}
	};
	std::vector<cBaseRender::tTexInfo> mTexInfo;	// texture rendering info

	// pipeline methods
	void InitShader();
	void Reload();
	void UpdateCamera();

	// set uniform values
	void SetBool(const std::string & name, bool value) const;
	void SetVector(const std::string, const tVector & vector) const;
	void SetMatrix(const std::string, const tMatrix & mat) const;

	// test code, need deleted
	void AddTestCubeTex();

	// // Add subroutines
	// void AddTextureMesh();
	// void AddNonTextureMesh();

};