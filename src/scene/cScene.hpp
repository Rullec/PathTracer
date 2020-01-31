#pragma once
#include <memory>
#include <string>

class cScene 
{   
public:
    cScene();
    ~cScene();
    virtual void Init() = 0;
    virtual void Update() = 0;
	virtual void KeyEvent(int key, int scancode, int action, int mods) = 0;
	virtual void MouseMoveEvent(double xpos, double ypos) = 0;
	virtual void MouseButtonEvent(int button, int action, int mods) = 0;
	virtual void ScrollEvent(double offset) = 0;

protected:
    enum eSceneStatus{
        BeforeInit = 0,
        InitSucc,
        Destroied,
        ScenStatusNum
    };
    
    std::string mSceneStatusName[ScenStatusNum] = {
        "BeforeInit", 
        "InitSucc", 
        "Destoried" 
    };
    enum eSceneStatus mSceneStatus;
};