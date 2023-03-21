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
void render() {
    // update scene and render
    // wrapper_.Get_Lock().lock();
    // get framebuffer viewport
    viewport_ = {0, 0, 0, 0};
    glfwGetFramebufferSize(window_, &viewport_.width, &viewport_.height);
    mjv_updateScene(wrapper_.Get_Model(), wrapper_.Get_Data(), &opt_, NULL, &cam_, mjCAT_ALL, &scn_);
    // if(&con_)
    mjr_render(viewport_, &scn_, &con_);
    // wrapper_.Get_Lock().unlock();

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window_);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
    
    //set mouse hooks so it can be interactive
}

void keep_rendering() {
    while(should_render_) {
        render();
        // FIX!!!
        // typedef std::chrono::duration<int, std::ratio<1, 60>> frame_duration;
        // frame_end_time += std::chrono::duration_cast<std::chrono::seconds>(frame_duration(1));
        // std::this_thread::sleep_until(frame_end_time);

        // CAN USE SOMETHING LIKE THIS FOR THE WAITING INSTEAD??
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        // mjtNum simstart = d->time;
        // while( d->time - simstart < 1.0/60.0 )
        //     mj_step(m, d);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void rendering_loop(int loop_count) {
    //rendering_indefinitely = true: keep running
    for (int i = 0; i < loop_count; i++) {
        render();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}
//MAKE ANOTHER FN THAT WOULD SET IT TO FALSE AND STOP THE THREAD

MujocoRenderer(MujocoModel<Dimensions>& wrap) //removed const
        :   wrapper_(wrap),
            rendering_thread_{}//,
            // scn_(nullptr),
            // con_(nullptr)
    {
        std::cout << "Actually gets into the constructor" << std::endl;
        //set up window

        if (!glfwInit()) {
            mju_error("Could not initialize GLFW");
        }

        window_ = glfwCreateWindow(800, 600, "Test", NULL, NULL);
        glfwMakeContextCurrent(window_);
        glfwSwapInterval(1);
        mjv_defaultCamera(&cam_);
        // mjv_defaultPerturb(&pert_);
        mjv_defaultOption(&opt_);
        mjr_defaultContext(&con_);
        mjv_defaultScene(&scn_);
        // const mjModel* model_ptr = wrapper_.Get_Model();
        // if(model_ptr)
        //     std::cout << "Model pointer is not null" << std::endl;
        mjv_makeScene(wrapper_.Get_Model(), &scn_, 1000);                     // space for 1000 objects
        std::cout << "Works till here" << std::endl;
        mjr_makeContext(wrapper_.Get_Model(), &con_, mjFONTSCALE_100);     // model-specific context

        rendering_thread_ = std::thread{&MujocoRenderer::keep_rendering, this};

        std::cout << "Constructed successfully before the thread stuff" << std::endl;

        // rendering_thread_ = std::thread([this/*, &wrapper_ = wrapper_*/](){
        //     this->keep_rendering();
        // });

        std::cout << "Constructed successfully" << std::endl;

    }

    void set_should_render(bool status) {
        should_render_ = status;
    }

~MujocoRenderer(){
    // close GLFW, free visualization storage
    glfwTerminate();
    mjv_freeScene(&scn_);
    mjr_freeContext(&con_);
    
    //SHOULD THIS BE STD::this_thread.join() INSTEAD??
    // rendering_thread_.join();

}

private:
    mjrRect viewport_; 
    mjvScene scn_;                     //SHOULD THESE BE POINTERS??
    mjrContext con_;                   // SHOULD THESE BE POINTERS??
    GLFWwindow* window_;
    mjvCamera cam_;                      //abstract camera
    mjvPerturb pert_;                    // perturbation object
    mjvOption opt_;                      // visualization options

     MujocoModel<Dimensions>& wrapper_; //not const because mjv_updateScene changes this

    std::thread rendering_thread_;
    

    bool should_render_ = true;
};

} //namespace gtfo