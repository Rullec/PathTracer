#pragma once
#include <scene/cScene.hpp>
#include <render/cBaseRender.hpp>
#include <render/camera/cBaseCamera.hpp>

class cQuadViser;
struct tPolygon;
class cDrawScene :public cScene
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	cDrawScene(const std::string & config );
	~cDrawScene();
	void Init() override;
	void Update() override;
	void KeyEvent(int key, int scancode, int action, int mods) override;
	void MouseMoveEvent(double xpos, double ypos) override;
	void MouseButtonEvent(int button, int action, int mods) override;
	void ScrollEvent(double offset) override;

private:
	std::shared_ptr<cBaseRender> mRender;
	std::shared_ptr<cBaseCamera> mCamera;

	// rendering info
	void DrawAxis();
	void ParseConfig(const std::string & conf);

protected:
	bool mDataReload;
	virtual void DrawScene();
	bool AddObjToScene(std::shared_ptr<cBaseMesh> obj);
};