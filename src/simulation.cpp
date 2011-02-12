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

sim::Robot::~Robot() {
  if( bot ) {
    delete bot;
  }
}

Simulation * sim::create_simulation(robot::AbstractRobot * bot, const char *mapPath, const char *botPath, const char *sensLibPath) {
  Simulation * result = new Simulation();
  result->map = read_map(mapPath);
  robot::SensorFactory * sensors = new sim::SensorFactory(sensLibPath, result);
  robot::MotorFactory * motors = new sim::MotorFactory(result);
  result->bot.bot = bot;
  robot::MCL * mcl = new robot::MCL();
  bot->addSlam(mcl);
  read_robot(bot, botPath, sensors, motors);
  result->bot.position = result->map->start;
  result->bot.lastPosition = result->bot.position;
  result->bot.bot->setStart(result->map->start);
  mcl->initialize(result->map->start, 10, *result->map);
  delete motors;
  delete sensors;
  return result;
}

void sim::Simulation::step() {
#ifdef REAL_ROBOT
  curSim = this;
#endif
  bot.bot->act();
  
  // find the displacement in the robot's coordinate system
  float theta = bot.angularVelocity * deltaT;
  math::Ray disp;
  if( fabsf(bot.angularVelocity) > 0.01) {
    float radius = bot.velocity.mag() / bot.angularVelocity;
    disp = math::Ray(math::vec2(0, -radius), math::vec2(1, 0));
    disp.transform(math::getRotationMatrix(theta));
    disp += math::vec2(0, radius);
  }
  else {
    disp = math::Ray(math::vec2(bot.velocity.mag() * deltaT, 0), math::vec2(1,0));
  }
  
  // transform into the world coordinate system
  bot.lastPosition = bot.position;
  bot.position += disp;
  //bot.position.rotateAboutStart(math::vec2(cos(theta), sin(theta)).getRotationMatrix());
}
