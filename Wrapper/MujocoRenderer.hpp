#pragma once

// Standard libraries includes
#include <string>
#include <array>
#include <thread>
#include <mutex>

// Third-party dependencies
#include <mujoco/mujoco.h>
#include "../External/mujoco/build/_deps/glfw-src/include/GLFW/glfw3.h"

// Project-specific
#include "../Core/Models/DynamicsBase.hpp"

// Include the rest of the Mujoco Wrapper
#include "MujocoWrapper.hpp"


namespace gtfo {

template<unsigned int Dimensions>
class MujocoRenderer{
public:
void render() {
    // update scene and render
    wrapper_lock_.lock();
    mjv_updateScene(wrapper_.Get_Model(), wrapper_.Get_Data(), &opt_, &pert_, &cam_, mjCAT_ALL, &scn_);
    wrapper_lock_.unlock();
    mjr_render(viewport_, &scn_, &con_);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window_);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
}

void rendering_loop(int loop_count) {
    //rendering_indefinitely = true: keep running
    for (int i = 0; i < count; i++) {
        render();
        //ADD SLEEP HERE
    }
}
//MAKE ANOTHER FN THAT WOULD SET IT TO FALSE AND STOP THE THREAD

MujocoRenderer(MujocoWrapper<Dimensions>& wrap, int loop_count, ) //removed const
        :   window_(glfwCreateWindow(800, 600, "Demo", NULL, NULL)),
            wrapper_(wrap),
            rendering_thread_{}//,
            // scn_(nullptr),
            // con_(nullptr)
    {
        //set up window
        glfwMakeContextCurrent(window_);
        glfwSwapInterval(1);
        mjv_defaultCamera(&cam_);
        mjv_defaultPerturb(&pert_);
        mjv_defaultOption(&opt_);
        mjr_defaultContext(&con_);
        mjv_makeScene(wrapper_.Get_Model(), &scn_, 2000);                     // space for 2000 objects
        mjr_makeContext(wrapper_.Get_Model(), &con_, mjFONTSCALE_100);     // model-specific context

        // get framebuffer viewport
        viewport_ = {0, 0, 0, 0};
        glfwGetFramebufferSize(window_, &viewport_.width, &viewport_.height);

        rendering_thread_ = std::thread{&MujocoRenderer::render, rendering_loop(loop_count)};

    }

~MujocoRenderer(){
    // close GLFW, free visualization storage
    glfwTerminate();
    mjv_freeScene(&scn_);
    mjr_freeContext(&con_);
    
    rendering_thread_.join();

}

private:
    mjrRect viewport_; 
    mjvScene scn_;                     //SHOULD THESE BE POINTERS??
    mjrContext con_;                   // SHOULD THESE BE POINTERS??
    GLFWwindow* window_;
    mjvCamera cam_;                      //abstract camera
    mjvPerturb pert_;                    // perturbation object
    mjvOption opt_;                      // visualization options

     MujocoWrapper<Dimensions>& wrapper_; //not const because mjv_updateScene changes this

    std::thread rendering_thread_;
    std::mutex wrapper_lock_;
};

} //namespace gtfo