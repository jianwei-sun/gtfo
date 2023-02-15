#pragma once

// Standard libraries includes
#include <string>
#include <array>

// Third-party dependencies
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

// Project-specific
#include "../Core/Models/DynamicsBase.hpp"

// Include the rest of the Mujoco Wrapper
#include "MujocoWrapper.hpp"


namespace gtfo {

template<unsigned int Dimensions>
class MujocoRenderer : public DynamicsBase<Dimensions, mjtNum>{
public:
MujocoRenderer(MujocoWrapper* wrap)
        :   window_(glfwCreateWindow(800, 600, "Demo", NULL, NULL)),
            wrapper_(wrap)
    {
        //set up window
        glfwMakeContextCurrent(window_);
        glfwSwapInterval(1);
        mjv_defaultCamera(&cam_);
        mjv_defaultPerturb(&pert_);
        mjv_defaultOption(&opt_);
        mjr_defaultContext(&con_);
        mjv_makeScene(&scn_, 2000);                     // space for 2000 objects
        mjr_makeContext(wrapper.Get_Model(), &con_, mjFONTSCALE_100);     // model-specific context

        // get framebuffer viewport
        viewport_ = {0, 0, 0, 0};
        glfwGetFramebufferSize(window_, &viewport_.width, &viewport_.height);

    }

 void render() {
        // update scene and render
        mjv_updateScene(wrapper.Get_Model(), wrapper.Get_Data(), &opt_, &pert_, &cam_, mjCAT_ALL, &scn_);
        mjr_render(viewport_, &scn_, &con_);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window_);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }

~MujocoRenderer(){
    // close GLFW, free visualization storage
    glfwTerminate();
    mjv_freeScene(&scn_);
    mjr_freeContext(&con_);
}

private:
    mjrRect viewport_; 
    mjvScene* scn_;
    mjrContext* con_;
    GLFWwindow* window_;
    mjvCamera cam_;                      //abstract camera
    mjvPerturb pert_;                    // perturbation object
    mjvOption opt_;                      // visualization options
    mjvScene scn_;                       // abstract scene
    mjrContext con_;                     // custom GPU context

    MujocoWrapper* wrapper_;
};

} //namespace gtfo