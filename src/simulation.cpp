/*
 *  simulation.cpp
 *  sim
 */

#include <cmath>
#include "simulation.h"
#include "robot.h"
#include "simsensors.h"
#include "simmotors.h"
#include "geometryio.h"
#include "timers.h"
#include "slam/simslam.h"
using namespace sim;

World::~World() {
  if( bot ) {
    delete bot;
  }
}

void World::pause() {
  if( bot ) {
    bot->halt();
  }
}

void sim::Simulation::step() {
  curSim = this;
  
  // find the displacement in the robot's coordinate system
  float theta = simBot.angularVelocity * deltaT;
  math::Ray disp;
  if( fabsf(simBot.angularVelocity) > 0.01) {
    float radius = simBot.velocity.mag() / simBot.angularVelocity;
    disp = math::Ray(math::vec2(0, -radius), math::vec2(1, 0));
    disp.transform(math::getRotationMatrix(theta));
    disp += math::vec2(0, radius);
  }
  else {
    disp = math::Ray(math::vec2(simBot.velocity.mag() * deltaT, 0), math::vec2(1,0));
  }
  
  // transform into the world coordinate system
  simBot.lastPosition = simBot.position;
  simBot.position += disp;
  
  bot->act();
  
  time += deltaT;
}

void sim::RealTimeSimulation::step() {
  curSim = this;
  struct timeval nextTime = robot::time();
  deltaT = ((float)robot::time_diff(lastTime, nextTime))/1000000.0;
  lastTime = nextTime;
  
  // find the displacement in the robot's coordinate system
  float theta = simBot.angularVelocity * deltaT;
  math::Ray disp;
  if( fabsf(simBot.angularVelocity) > 0.01) {
    float radius = simBot.velocity.mag() / simBot.angularVelocity;
    disp = math::Ray(math::vec2(0, -radius), math::vec2(1, 0));
    disp.transform(math::getRotationMatrix(theta));
    disp += math::vec2(0, radius);
  }
  else {
    disp = math::Ray(math::vec2(simBot.velocity.mag() * deltaT, 0), math::vec2(1,0));
  }
  
  // transform into the world coordinate system
  simBot.lastPosition = simBot.position;
  simBot.position += disp;
  
  bot->act();
}

void RealTimeSimulation::start() {
  lastTime = robot::time();
}

RealWorld::RealWorld(robot::AbstractRobot * b)
: World(b->getPosition(), b) {
}

void RealWorld::step() {
  bot->act();
}
