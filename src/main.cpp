#include <iostream>
#include <scene/BuildScene.hpp>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <fstream>

int gWindowWidth = 800;
int gWindowHeight = 600;
int gStartX = 100;
int gStartY = 100;
std::string gWindowName = "";
std::shared_ptr<cScene> gScene = nullptr;
GLFWwindow * gWindow = nullptr;

void InitGL();
void InitGLFW();
void ParseConfig(const std::string & conf);

void MouseButtonEventCallback(GLFWwindow* window, int button, int action, int mods);
void MouseMoveEventCallback(GLFWwindow* window, double xpos, double ypos);
void ErrorCallback(int error, const char* description);
void KeyEventCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
void ResizeCallback(GLFWwindow* window, int w, int h);
void ScrollCallback(GLFWwindow* window, double xoff, double yoff);

#include <algorithm>
int main()
{
	std::string conf = "config/path.conf";
	ParseConfig(conf);
	InitGLFW();
	InitGL();

	// init scene
	gScene = BuildScene(conf);
	gScene->Init();
	while (!glfwWindowShouldClose(gWindow))
	{
		gScene->Update();

		//std::cout << "update" << std::endl;
		glfwSwapBuffers(gWindow);
		glfwPollEvents();
	}
	glfwTerminate();
}


void MouseMoveEventCallback(GLFWwindow* window, double xpos, double ypos)
{
	//std::cout << "[log] mouse move to " << xpos << " " << ypos << std::endl;
	gScene->MouseMoveEvent(xpos, ypos);
}

void MouseButtonEventCallback(GLFWwindow* window, int button, int action, int mods)
{
	gScene->MouseButtonEvent(button, action, mods);
}

void ErrorCallback(int error, const char* description)
{
	std::cout << "[error] GLFW error callback: " << error << " " << description << std::endl;
	exit(1);
}

void KeyEventCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GLFW_TRUE);

	gScene->KeyEvent(key, scancode, action, mods);
}

void InitGL()
{
	if (GLEW_OK != glewInit())
	{
		std::cout << "[errpr] glew init failed " << std::endl;
		exit(1);
	}

	glEnable(GL_DEPTH_TEST);
	glClearColor(0.2, 0.3, 0.4, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void InitGLFW()
{
	// init glfw
	if (!glfwInit())
	{
		std::cout << "[error] InitGLFW:: glfw inti failed" << std::endl;
		glfwTerminate();
	}
	glfwSetErrorCallback(ErrorCallback);

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);// fixed size

	gWindow = glfwCreateWindow(gWindowWidth, gWindowHeight, gWindowName.c_str(), NULL, NULL);
	if (gWindow == NULL)
	{
		std::cout << "[error] Failed to create GLFW window" << std::endl;
		glfwTerminate();
	}
	glfwSetWindowPos(gWindow, gStartX, gStartY);
	glfwMakeContextCurrent(gWindow);

	glfwSetKeyCallback(gWindow, KeyEventCallback);
	glfwSetCursorPosCallback(gWindow, MouseMoveEventCallback);
	glfwSetMouseButtonCallback(gWindow, MouseButtonEventCallback);
	glfwSetInputMode(gWindow, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
	glfwSetFramebufferSizeCallback(gWindow, ResizeCallback);
	glfwSetScrollCallback(gWindow, ScrollCallback);
}

void ParseConfig(const std::string & conf)
{
	Json::Value root;
	cJsonUtil::ParseJson(conf, root);

	// MainWindowInfo request
	Json::Value mainwindow_info = root["MainWindowInfo"];
	gStartX = mainwindow_info["StartX"].asInt();
	gStartY = mainwindow_info["StartY"].asInt();
	gWindowWidth = mainwindow_info["Width"].asInt();
	gWindowHeight = mainwindow_info["Height"].asInt();
	gWindowName = mainwindow_info["WindowName"].asString();
}

void ResizeCallback(GLFWwindow* window, int w, int h)
{
	gScene->Update();
	glfwSwapBuffers(gWindow);
}

void ScrollCallback(GLFWwindow* window, double xoff, double yoff)
{
	//std::cout << "scroll: x y = " << xoff << " " << yoff << std::endl;
	gScene->ScrollEvent(yoff);
}