#pragma once

// Standard libraries includes
#include <string>
#include <array>
#include <thread>
#include <mutex>
#include <chrono>
#include <iostream>

// Third-party dependencies
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

// Project-specific
#include "../Core/Models/DynamicsBase.hpp"

// Include the rest of the Mujoco Wrapper
#include "../Core/Models/Mujoco/MujocoModel.hpp"

#define _GLIBCXX_USE_NANOSLEEP //for the thread sleep function

namespace gtfo {

template<unsigned int Dimensions>
class MujocoRenderer{
public:

// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left_ && !button_middle_ && !button_right_ )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx_;
    double dy = ypos - lasty_;
    lastx_ = xpos;
    lasty_ = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right_ )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left_ )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(wrapper_.Get_Model(), action, dx/height, dy/height, &scn_, &cam_);
}

// mouse button callback
// the function doesn't actually modify any of the last 3 parameters- would only need them if we used the callback
void mouse_button(GLFWwindow* window/*, int button, int act, int mods*/)
{
    // update button state
    button_left_ =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right_ =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx_, &lasty_);
}

// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(wrapper_.Get_Model(), mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn_, &cam_);
}

void render() {
    // wrapper_.Get_Lock().lock();

    // get framebuffer viewport
    viewport_ = {0, 0, 0, 0};
    glfwGetFramebufferSize(window_, &viewport_.width, &viewport_.height);
    mjv_updateScene(wrapper_.Get_Model(), wrapper_.Get_Data(), &opt_, NULL, &cam_, mjCAT_ALL, &scn_);
    mjr_render(viewport_, &scn_, &con_);
    
    // wrapper_.Get_Lock().unlock();

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window_);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();

    // install GLFW mouse and keyboard callbacks
    double xpos, ypos;
    glfwGetCursorPos(window_, &xpos, &ypos);
    mouse_move(window_, xpos, ypos);
    mouse_button(window_);
    scroll(window_, xpos, ypos);
}

void keep_rendering(void) {
    if (!glfwInit()) {
            mju_error("Could not initialize GLFW");
    }

    // #ifdef __APPLE__
    /* We need to explicitly ask for a 3.3 context on OS X */
    glfwWindowHint (GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint (GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint (GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint (GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    // #endif

    window_ = glfwCreateWindow(800, 600, "Test", NULL, NULL);
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);
    mjv_defaultCamera(&cam_);
    mjv_defaultOption(&opt_);
    mjr_defaultContext(&con_);
    mjv_defaultScene(&scn_);
    mjv_makeScene(wrapper_.Get_Model(), &scn_, 1000);                     // space for 1000 objects
    mjr_makeContext(wrapper_.Get_Model(), &con_, mjFONTSCALE_100);        // model-specific context

    while(should_render_) {
        render();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // free visualization storage, close GLFW
    mjv_freeScene(&scn_);
    mjr_freeContext(&con_);
    glfwTerminate();
}

MujocoRenderer(MujocoModel<Dimensions>& wrap)
        :   wrapper_(wrap), rendering_thread_{}
{
    rendering_thread_ = std::thread{&MujocoRenderer::keep_rendering, this};
}

void set_should_render(bool status) {
    should_render_ = status;
}

~MujocoRenderer(){
    rendering_thread_.join();

}

private:
    mjrRect viewport_;
    mjrContext con_;
    GLFWwindow* window_;
    mjvPerturb pert_;                    // perturbation object
    mjvOption opt_;                      // visualization options

    MujocoModel<Dimensions>& wrapper_; //not const because mjv_updateScene changes this

    std::thread rendering_thread_;
    
    bool should_render_ = true;

    // mouse interaction
    bool button_left_ = false;
    bool button_middle_ = false;
    bool button_right_ =  false;
    double lastx_ = 0;
    double lasty_ = 0;

    mjvScene scn_;
    mjvCamera cam_;                      //abstract camera
};

} //namespace gtfo