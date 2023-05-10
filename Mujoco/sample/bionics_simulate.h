#ifndef BIONICS_SIMULATE_H
#define BIONICS_SIMULATE_H

#include <cstring>
#include <vector>

int foo(const char* xmlDir);

// a limb
class SimObject {
public:
  SimObject(int groupNum);
  void update(float rotationPercentage);

private:
  int m_groupNum;
  double m_rotationPercentage; // 0 to 1

  friend class Simulation;
};

// the main simulation class
class Simulation {
public:
  Simulation(const char* modelFileName, std::vector<SimObject*> simObjects);
  ~Simulation();
  void step();
  bool isAlive();

private:
  std::vector<SimObject*> simObjects;
  bool m_isAlive;
};

#endif // BIONICS_SIMULATE_H