#pragma once
#include <scene/cScene.hpp>
#include <render/cBaseRender.hpp>
#include <render/cFocusCamera.h>

class cQuadViser;
struct tPolygon;
class cDrawScene :public cScene
{
public:
	cDrawScene(const std::string & config );
	~cDrawScene();
	void Init() override final;
	void Update() override final;
	void KeyEvent(int key, int scancode, int action, int mods) override final;
	void MouseMoveEvent(double xpos, double ypos) override final;
	void MouseButtonEvent(int button, int action, int mods) override final;
	void ScrollEvent(double offset) override final;
private:
	std::shared_ptr<cBaseRender> mRender;
	std::shared_ptr<cFocusCamera> mCamera;
	std::vector<tPolygon> mFaces;

	// rendering info
	int mCurFrame, mCurFrameWidth, mCurFrameHeight;
	bool mCameraActive;
	double mCursorLastX, mCursorLastY;
	bool mDataReload;

	void DrawScene();
	void DrawAxis();
	void ParseConfig(const std::string & conf);
};