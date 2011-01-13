/*
 *  robot.cpp
 *  sim
 */

#include <iostream>
#include <fstream>
#include <string>
#include "robot.h"
#include "geometryio.h"
using namespace robot;
using namespace math;

const std::string Robot::FRONT("FRONT");
const std::string Robot::BACK("BACK");
const std::string Robot::LEFT("LEFT");
const std::string Robot::RIGHT("RIGHT");

Robot * robot::read_robot(const char * path, SensorFactory * sensors, MotorFactory * motors) {
  std::map<std::string, math::Ray> sensorPositions;
  std::ifstream input(path);
  Robot * bot = new Robot;
  std::string astring;
  while( input ) {
    input >> astring;
    if( astring[0] == '#' ) { // comment, skip line
      input.ignore(LONG_MAX, '\n');
    }
    else if( astring == "radius" ) {
      input >> bot->size;
    }
    else if( astring == "position" ) {
      math::Ray pos;
      input >> astring;
      input >> pos;
      sensorPositions[astring] = pos;
    }
    else if( astring == "sensor" ) {
      std::string position;
      input >> position;
      input >> astring;
      RangeSensor * ranger = sensors->rangeSensor(astring.c_str());
      bot->rangeFinders[position] = ranger;
      ranger->relPos = sensorPositions[position];
    }
  }
  
  input.close();
  
  bot->motors = motors->newMotors();
  
  return bot;
}

Robot::Robot() :rangeFinders(), motors(), edges(), path(),
                position()
{
}

void robot::Robot::act() {
  edges.push_back(position.transformToAbsolute(rangeFinders[FRONT]->getCoordinate()));
  edges.push_back(position.transformToAbsolute(rangeFinders[BACK]->getCoordinate()));
  edges.push_back(position.transformToAbsolute(rangeFinders[LEFT]->getCoordinate()));
  edges.push_back(position.transformToAbsolute(rangeFinders[RIGHT]->getCoordinate()));
  
  float velocity = 10;
  float angularVelocity = 0;//M_PI/4.0;
  float deltaT = 0.1;
  motors->setVelocity(velocity);
  motors->setAngularVelocity(angularVelocity);
  path.push_back(position.origin());
  //math::vec2 velocityVec(velocity, 0);
  
  float theta = angularVelocity * deltaT;
  math::Ray disp;
  if( fabsf(angularVelocity) > 0.01) {
    float radius = velocity / angularVelocity;
    disp = math::Ray(math::vec2(0, -radius), math::vec2(1, 0));
    disp.transform(math::getRotationMatrix(theta));
    disp += math::vec2(0, radius);
  }
  else {
    disp = math::Ray(math::vec2(velocity * deltaT, 0), math::vec2(1, 0));
  }
  
  // transform into the world coordinate system
  position += disp;
  //position.rotateAboutStart(math::vec2(cos(theta), sin(theta)).getRotationMatrix());
  
}
