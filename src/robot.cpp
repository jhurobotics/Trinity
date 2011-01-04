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

Robot * robot::read_robot(const char * path, SensorFactory * sensors, MotorFactory * motors) {
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
    else if( astring == "sensor" ) {
      input >> astring;
      const math::Ray* pos = NULL;
      if( astring == "NORTH") {
        pos = &bot->NORTH;
      }
      else if( astring == "SOUTH") {
        pos = &bot->SOUTH;
      }
      else if( astring == "EAST" ) {
        pos = &bot->EAST;
      }
      else if( astring == "WEST" ) {
        pos = &bot->WEST;
      }
      
      if( pos ) {
        input >> astring;
        RangeSensor * ranger = sensors->rangeSensor(astring.c_str());
        bot->rangeFinders[*pos] = ranger;
        ranger->relPos = *pos;
      }
    }
  }
  
  input.close();
  
  bot->motors = motors->newMotors();
  
  return bot;
}

Robot::Robot() : NORTH(math::vec2(0, 4), math::vec2(0, 1)),
                SOUTH(math::vec2(0, -4), math::vec2(0, -1)),
                EAST(math::vec2(4, 0), math::vec2(1, 0)),
                WEST(math::vec2(-4, 0), math::vec2(-1, 0)),
                position(), edges()
{
}

void robot::Robot::act() {
  edges.push_back(position.transformToAbsolute(NORTH.getPoint(rangeFinders[NORTH]->getValue())));
  edges.push_back(position.transformToAbsolute(SOUTH.getPoint(rangeFinders[SOUTH]->getValue())));
  edges.push_back(position.transformToAbsolute(EAST.getPoint(rangeFinders[EAST]->getValue())));
  edges.push_back(position.transformToAbsolute(WEST.getPoint(rangeFinders[WEST]->getValue())));
  
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