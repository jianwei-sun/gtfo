// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstdio>
#include <cstring>
#include <iostream>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include "bionics_simulate.h"

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

GLFWwindow* window;

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

SimObject::SimObject(int groupNum) : m_groupNum(groupNum), m_rotationPercentage(0.5){
}

void SimObject::update(float rotationPercentage) {
  m_rotationPercentage = rotationPercentage;
}

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
  // backspace: reset simulation
  if (act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE) {
    mj_resetData(m, d);
    mj_forward(m, d);
  }
}


// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
  // update button state
  button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
  button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos) {
  // no buttons down: nothing to do
  if (!button_left && !button_middle && !button_right) {
    return;
  }

  // compute mouse displacement, save
  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if (button_right) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (button_left) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  // move camera
  mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}


Simulation::Simulation(const char* modelFileName, std::vector<SimObject*> simObjects)
  : m_isAlive(true)
{
  this->simObjects = simObjects;
  std::printf("starting bionics mujoco simulator\n");

  char error[1000] = "Could not load binary model";
  m = mj_loadXML(modelFileName, 0, error, 1000);
  if (!m) {
    mju_error("Load model error: %s", error);
  }


  // make data
  d = mj_makeData(m);

  // init GLFW
  if (!glfwInit()) {
    mju_error("Could not initialize GLFW");
  }

  // create window, make OpenGL context current, request v-sync
  window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  // install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(window, keyboard);
  glfwSetCursorPosCallback(window, mouse_move);
  glfwSetMouseButtonCallback(window, mouse_button);
  glfwSetScrollCallback(window, scroll);

  // run main loop, target real-time simulation and 60 fps rendering
  mj_step(m, d);
  printf("data output\n");
  printf("njnt:\t%d\n",m->njnt);
  for (int i = 0; i < m->njnt; i++) {
    printf("njnt #%d:\t qpos:%d\tjnt group:%d\t\n", i, m->jnt_qposadr[i], m->jnt_group[i]);
  }
}

Simulation::~Simulation() {
  //free visualization storage
  mjv_freeScene(&scn);
  mjr_freeContext(&con);

  // free MuJoCo model and data
  mj_deleteData(d);
  mj_deleteModel(m);

  // terminate GLFW (crashes with Linux NVidia drivers)
  #if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
  #endif

}
bool Simulation::isAlive() {
  return m_isAlive;
}

void Simulation::step() {
  if (glfwWindowShouldClose(window)) {
    m_isAlive = false;
    return;
  }

  // advance interactive simulation for 1/60 sec
  //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
  //  this loop will finish on time for the next frame to be rendered at 60 fps.
  //  Otherwise add a cpu timer and exit this loop when it is time to render.
  mjtNum simstart = d->time;
  while (d->time - simstart < 1.0/60.0) {
    d->time += m->opt.timestep;
    mj_kinematics(m, d);// TODO: remove this
    
    //printf("%lu\n", simObjects.size());
    for (int i = 0; i < simObjects.size(); i++) {
      //printf("%d: %d\n", i, simObjects[i]->m_groupNum);
      for (int j = 0; j < m->njnt; j++) {
        // find matching group number
        if (m->jnt_group[j] == simObjects[i]->m_groupNum) {
          d->qpos[m->jnt_qposadr[j]] = simObjects[i]->m_rotationPercentage;
        }
      }
    }
  }

  // get framebuffer viewport
  mjrRect viewport = {0, 0, 0, 0};
  glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

  // update scene and render
  mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
  mjr_render(viewport, &scn, &con);

  // swap OpenGL buffers (blocking call due to v-sync)
  glfwSwapBuffers(window);

  // process pending GUI events, call GLFW callbacks
  glfwPollEvents();

}