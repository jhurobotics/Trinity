/*
 *  simulation.cpp
 *  sim
 */

#include <cmath>
#include <iostream>
#include "simulation.h"
#include "robot.h"
#include "simsensors.h"
#include "simmotors.h"
#include "geometryio.h"
using namespace sim;

Simulation * sim::create_simulation(const char *mapPath, const char *botPath, const char *sensLibPath) {
  Simulation * result = new Simulation();
  result->map = read_map(mapPath);
  robot::SensorFactory * sensors = new sim::SensorFactory(sensLibPath, result);
  robot::MotorFactory * motors = new sim::MotorFactory(result);
  result->bot.bot = read_robot(botPath, sensors, motors);
  result->bot.position.setOrigin(result->map->start);
  result->bot.position.setDir(math::vec2(0, -1));
  delete motors;
  delete sensors;
  return result;
}

void sim::Simulation::step() {
  bot.bot->act();
  
  // find the displacement in the robot's coordinate system
  float theta = bot.angularVelocity * deltaT;
  std::cout << "theta = " << theta*180/M_PI << "\n";
  math::Ray disp;
  if( fabsf(bot.angularVelocity) > 0.01) {
    float radius = bot.velocity.mag() / bot.angularVelocity;
    disp = math::Ray(math::vec2(0, -radius), math::vec2(1, 0));
    disp.transform(math::getRotationMatrix(theta));
    disp += math::vec2(0, radius);
    std::cout << "displacement: " << disp << "\n";
  }
  else {
    disp = math::Ray(math::vec2(bot.velocity.mag() * deltaT, 0), math::vec2(1,0));
  }
  
  // transform into the world coordinate system
  bot.position += disp;
  std::cout << "bot at : " << bot.position << "\n";
  //bot.position.rotateAboutStart(math::vec2(cos(theta), sin(theta)).getRotationMatrix());
}