#include "cFPSCamera.hpp"
#include <util/cJsonUtil.hpp>
#include <iostream>
#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>
#include <util/cGlmUtil.hpp>

cFPSCamera::cFPSCamera(const std::string & conf) : cBaseCamera(conf)
{
    // std::cout <<"FPS camera constructed \n"; 
    ParseConf(conf);
    // std::cout << "pos = " << mCameraPos.transpose( ) << std::endl;
    // std::cout << "front = " << mCameraFront.transpose( ) << std::endl;
    // std::cout << "up = " << mCameraRestUp.transpose( ) << std::endl;
    Reload();
}

void cFPSCamera::ParseConf(const std::string & conf)
{
    Json::Value root;
    cJsonUtil::ParseJson(conf, root);
}

void cFPSCamera::Reload()
{
    // std::cout <<"[debug] camera fps reload " << std::endl;
    // std::cout <<"front = " << mCameraFront.transpose() << std::endl;
    // std::cout <<"up = " << mCameraUp.transpose() << std::endl;

    // compute view trans
    {
        glm::vec3 pos = cGlmUtil::tVectorToGlmVector3(mCameraPos),
                front = cGlmUtil::tVectorToGlmVector3(mCameraFront),
                up = cGlmUtil::tVectorToGlmVector3(mCameraUp);
        glm::mat4 view_mat = glm::lookAt(pos, pos + front, up);
        mViewTrans = cGlmUtil::GlmMatixTotMatrix(view_mat);
    }
    // mViewTrans = tMatrix::Identity();

    // calculate projection 
    {
        // glm::mat4 projection = glm::perspective(glm::radians(fov), 800.0f / 600.0f, 0.1f, 100.0f); 
        // mProjTrans = cGlmUtil::GlmMatixTotMatrix(projection);
        glm::mat4 projection = glm::perspective(glm::radians(mFOV), mWindowWidth / mWindowHeight, 0.1f, static_cast<float>(1e4)); 
        mProjTrans = cGlmUtil::GlmMatixTotMatrix(projection);
    }
    // mProjTrans = tMatrix::Identity();
    mRenderMat =  mProjTrans * mViewTrans;
    // mRenderMat.transposeInPlace();
}

void cFPSCamera::MouseMoveEvent(double xpos, double ypos)
{
    // std::cout << xpos <<" " << ypos << std::endl;
    if(mCameraActive == true && false == std::isnan(mCursorLastY) && false == std::isnan(mCursorLastX))
    {
        // std::cout <<"-----------------------\n";
        double move_vel = 2e-3;
        tVector move_vec = tVector(xpos - mCursorLastX, ypos - mCursorLastY, 0, 0) * move_vel;
        // std::cout <<"raw move vec = " << move_vec.transpose() << std::endl;
        move_vec[1] *=-1;
        // std::cout <<"move vec in tranditional RH frame = " << move_vec.transpose() << std::endl;
        move_vec = cMathUtil::RotMat(cMathUtil::AxisAngleToQuaternion(tVector(0, 0, 1, 0) * cMathUtil::Radians(90))) * move_vec; 
        // std::cout <<"move vec after 90 d= " << move_vec.transpose() << std::endl;
        move_vec = mViewTrans.transpose() * move_vec;
        // std::cout <<"move vec in world coor = " << move_vec.transpose() << std::endl;
        tMatrix rot_mat = cMathUtil::RotMat(cMathUtil::AxisAngleToQuaternion(move_vec));
        rot_mat.transposeInPlace();
        // std::cout <<"rot mat = " << rot_mat << std::endl;

        mCameraFront = rot_mat * mCameraFront;
        mCameraUp = rot_mat * mCameraUp;
        if(true == std::isnan(mCameraUp.maxCoeff()))
        {
            std::cout <<"move vec in world frame = " << move_vec.transpose() << std::endl;
            std::cout <<"rot mat = " << rot_mat.transpose() << std::endl;
            exit(1);
        }
    }

    cBaseCamera::MouseMoveEvent(xpos, ypos);

    Reload();
}

void cFPSCamera::MouseButtonEvent(int button, int action, int mods)
{
    // std::cout <<"[debug] fps button " << std::endl;
    cBaseCamera::MouseButtonEvent(button, action, mods);
}

void cFPSCamera::KeyEvent(int key, int scancode, int action, int mods)
{
    if(action == GLFW_PRESS || action == GLFW_REPEAT)
    {
        double move_vel = 2e-1;
        switch (key)
        {
        case GLFW_KEY_W:
            mCameraPos += mCameraFront * move_vel;
            break;
        case GLFW_KEY_A:
            mCameraPos -= mCameraFront.cross3(mCameraUp) * move_vel;
            break;
        case GLFW_KEY_S:
            mCameraPos -= mCameraFront * move_vel;
            break;
        case GLFW_KEY_D:
            mCameraPos += mCameraFront.cross3(mCameraUp) * move_vel;
            break;
        case GLFW_KEY_SPACE:
            mCameraPos += mCameraUp * move_vel;
            break;
        case GLFW_KEY_Z:
            mCameraPos -= mCameraUp * move_vel;
        default:
            break;
        }
    }
    Reload();
	
    // std::cout <<"[debug] fps keyevent " << std::endl;
}

void cFPSCamera::ScrollEvent(double offset)
{
    // std::cout <<"[debug] fps scroll  " << std::endl;
}