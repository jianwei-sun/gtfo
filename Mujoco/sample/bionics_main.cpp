#include "bionics_simulate.h"

#include <cstdio>
#include <cstring>
#include <iostream>
/*
int main(int argc, const char** argv) {
}*/

class HAL {
  public:
    HAL(const char* xmlDir);
    void step();
  private:
    std::vector<SimObject*> simObjects;
    Simulation* s;
    float a = 0;
};

HAL::HAL(const char* xmlDir) {
  // create 7 arms
  for (int i = 0; i < 7; i++) {
    simObjects.push_back(new SimObject(i));
  }
  s = new Simulation(xmlDir, simObjects);
}

void HAL::step() {
  a += 0.01;
  // give the simObjects the new positions/rotations of the arm
  simObjects[1]->update(a);
  simObjects[2]->update(1-a);
  simObjects[3]->update(a);
  if (a > 1) a = 0;

  // finally, step the simulation to show the result
  if (s->isAlive()) {
    s->step();
  } else {
    delete s; // TODO: probably not safe
  }
}

// main function
int main(int argc, const char** argv) {
  // check command-line arguments
  if (argc!=2) {
    std::printf(" USAGE:  basic modelfile\n");
    return 0;
  }

  HAL* hal = new HAL(argv[1]);
  while(true) {
    hal->step();
  }

}