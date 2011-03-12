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

Simulation * sim::create_simulation(robot::AbstractRobot * bot, const char *mapPath, const char *botPath, const char *sensLibPath) {
  Simulation * result = new Simulation();
  result->map = read_map(mapPath);
  robot::SensorFactory * sensors = new sim::SensorFactory(sensLibPath, result);
  robot::MotorFactory * motors = new sim::MotorFactory(result);
  result->bot = bot;
  robot::MCL * mcl = new robot::MCL();
  bot->addSlam(mcl);
  read_robot(bot, botPath, sensors, motors);
  result->simBot.position = result->map->start;
  result->simBot.lastPosition = result->simBot.position;
  mcl->initialize(result->map->start, 10, *result->map);
  delete motors;
  delete sensors;
  return result;
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
}

void sim::RealTimeSimulation::step() {
  curSim = this;
  unsigned long nextTime = robot::milli_time();
  deltaT = (nextTime - lastTime)/1000.0;
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
  lastTime = robot::milli_time();
}

RealWorld::RealWorld(robot::AbstractRobot * b)
: World(b->getPosition(), b) {
}

void RealWorld::step() {
  bot->act();
}
