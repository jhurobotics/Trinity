/*
 *  robot.cpp
 *  sim
 */

#include <climits>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>
#include <boost/python.hpp>
#include "robot.h"
#include "geometryio.h"
#ifndef __APPLE__
#include <GL/gl.h>
#else
#include <OpenGL/gl.h>
#endif
using namespace robot;
using namespace math;

static AbstractRobot * python_robot(const std::string& pyName) {
  return boost::python::extract<AbstractRobot*>(boost::python::eval((pyName+"()").c_str()));
}

AbstractRobot * robot::new_robot(robot::Implementation imp, const char * path) throw(BadRobotImplementation) {
  switch( imp ) {
  case PYTHON:
    return python_robot(path);
  case SONAR:
    return new SonarRobot;
  case CPP_1:
  case CPP_2:
  default:
    throw BadRobotImplementation();
  }
}

typedef std::map<std::string, math::Ray> StrToRay_t;

static Ray read_position(std::istream& input, StrToRay_t& sensorPositions) {
  Ray posRay;
  std::string position;
  input >> position;
  if( sensorPositions.find(position) != sensorPositions.end() ) {
    return sensorPositions[position];
  }
  else {
    float x, y;
    x = atof(position.c_str());
    input >> y;
    vec2 pos(x,y);
    vec2 dir;
    input >> dir;
    return Ray(pos, dir);
  }
}

void robot::read_robot(AbstractRobot * bot, const char * path, SensorFactory * sensors, MotorFactory * motors) {
  enum SensorType {
    UNKNOWN = 0,
    RANGE,
    ENCODER
  };
  SensorType curSensType = UNKNOWN;
  StrToRay_t sensorPositions;
  std::ifstream input(path);
  std::string astring;
  while( input ) {
    input >> astring;
    if( astring[0] == '#' ) { // comment, skip line
      input.ignore(LONG_MAX, '\n');
    }
    else if( astring == "radius" ) {
      input >> astring; // ignored
    }
    else if( astring == "position" ) {
      math::Ray pos;
      input >> astring;
      input >> pos;
      sensorPositions[astring] = pos;
    }
    else if( astring == "range:" ) {
      curSensType = RANGE;
    }
    else if( astring == "encoder:" ) {
      curSensType = ENCODER;
    }
    else if( astring == "sensor" ) {
      switch( curSensType ) {
        case UNKNOWN:
          break;
        case RANGE: {
          Ray posRay = read_position(input, sensorPositions);
          input >> astring;
          RangeSensor * ranger = sensors->rangeSensor(astring.c_str());
          ranger->relPos = posRay;
          bot->addRangeSensor(ranger);
          break;
        }
        case ENCODER: {
          vec2 pos = read_position(input, sensorPositions).origin();
          input >> astring;
          Encoder * encoder = sensors->encoder(astring.c_str());
          encoder->relPos = pos;
          bot->addEncoder(encoder);
          break;
        }
      }
    }
  }
    
  input.close();
  
  bot->addMotors(motors->newMotors());
  bot->addGraph(new Graph());
  //bot->addSlam(new SLAM());
  
  //return bot;
}

SonarRobot::SonarRobot() throw() : rangeFinders(), motors(), edges(), path(),
                         position()
{
}

void robot::SonarRobot::act() throw() {
  typedef std::set<RangeSensor*>::iterator RangeIterator;
  RangeIterator end = rangeFinders.end();
  for( RangeIterator iter = rangeFinders.begin(); iter != end; iter ++ ) {
    edges.push_back(position.transformVecToAbsolute((*iter)->getCoordinate()));
  }
  
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
  
  // update the slammer
  position = slammer->getPose();
}

#define RED 1.0, 0.0, 0.0
#define GREEN 0.0, 1.0, 0.0
#define BLUE 0.0, 0.0, 1.0
#define WHITE 1.0, 1.0, 1.0

void SonarRobot::draw() {
  glRotatef(-90.0, 0.0, 0.0, 1.0);
  glPointSize(4.0);
  glColor4f(BLUE, 1.0);
  glBegin(GL_POINTS);
  for( unsigned int i = 0; i < path.size(); i++ ) {
    glVertex2f(path[i].x, path[i].y);
  }
  glColor4f(GREEN, 1.0);
  for( unsigned int i = 0; i < edges.size(); i++ ) {
    glVertex2f(edges[i].x, edges[i].y);
  }
  glEnd();
  
  slammer->draw();
}
