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
#include "arduino.h"
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

AbstractRobot::~AbstractRobot() {
  if( graph ) {
    delete graph;
  }
  if( slammer ) {
    delete slammer;
  }
  if( arduino ) {
    delete arduino;
  }
  //if( maestro ) {
  //  delete maestro;
  //}
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
}

SonarRobot::SonarRobot() throw() : curMode(HALLWAY), currentObjective(NULL), rangeFinders(),
                                   motors(), edges(), path(), position()
{
}

void robot::SonarRobot::act() throw() {
  // update the slammer
  position = slammer->getPose();
  
  path.push_back(position.origin());
  typedef std::set<RangeSensor*>::iterator RangeIterator;
  RangeIterator end = rangeFinders.end();
  for( RangeIterator iter = rangeFinders.begin(); iter != end; iter ++ ) {
    edges.push_back(position.transformVecToAbsolute((*iter)->getCoordinate()));
  }
  
  // Only move once the SLAM module has settled
  if( slammer->settled() ) {
    hallway();
  }
    
}

#define MOVE_SPEED 20
#define TURN_SPEED M_PI / 8.0
const static float ANGLE_RES = M_PI/20.0;

void SonarRobot::hallway() throw() {
  static bool moving = false;
  if( !currentObjective ) {
    // get an objective!
    currentObjective = graph->getObjective(position.origin());
  }
  
  // am I at the objective?
  if( currentObjective->loc.contains(position.origin()) ) {
    motors->setVelocity(0);
    motors->setAngularVelocity(0);
    moving = false;
    //if( !currentObjective->checked ) {
    //  curMode = SCAN;
    //}
    //else {
    currentObjective = graph->getObjective(position.origin());
    //}
    
  }
  else { // get there
    vec2 disp = currentObjective->loc.center - position.origin();
      
    float dispDir = atan2(disp.y, disp.x);
    float curDir = position.angle();
    if( abs(std::fmod((double)dispDir - curDir, 2*M_PI)) < ANGLE_RES / (moving ? 1 : 2)
        || abs(std::fmod((double)dispDir - curDir, 2*M_PI)) > 2 * M_PI - (ANGLE_RES / (moving ? 1 : 2)) ) {
      motors->setVelocity(MOVE_SPEED);
      motors->setAngularVelocity(0);
      moving = true;
    }
    else {
      if( moving )
        motors->setVelocity(0);
      float delta = dispDir - curDir;
      if( delta > M_PI ) {
        delta -= 2*M_PI;
      }
      else if( delta < -M_PI) {
        delta += 2*M_PI;
      }
      motors->setAngularVelocity(TURN_SPEED * (delta > 0 ? 1 : -1 ));
    }
  }
}


#define RED 1.0, 0.0, 0.0
#define GREEN 0.0, 1.0, 0.0
#define BLUE 0.0, 0.0, 1.0
#define WHITE 1.0, 1.0, 1.0
#define THOUGHT 1.0, 0.5, 0.5

void SonarRobot::draw() {
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
  
  glPushMatrix();
  glTranslatef(position.origin().x, position.origin().y, 0.0);
  glRotatef((atan2(position.dir().y, position.dir().x))*180.0/M_PI, 0.0, 0.0, 1.0);
  glColor4f(THOUGHT, 1.0);
  glBegin(GL_LINE_LOOP);
  int angleCount = 32;
  for(int i = 0; i < angleCount; i++) {
    glVertex2f(4*cos(((float)i)/((float)angleCount)*2*M_PI), 4*sin(((float)i)/((float)angleCount)*2*M_PI));
  }
  glEnd();
  glBegin(GL_LINES);
  glVertex2f(0.0, 0.0);
  glVertex2f(8.0, 0.0);
  glVertex2f(6.0, -2.0);
  glVertex2f(8.0, 0.0);
  glVertex2f(8.0, 0.0);
  glVertex2f(6.0, 2.0);
  glEnd();

  glPopMatrix();
  
  if( currentObjective ) {
    glPushMatrix();
    glTranslatef(currentObjective->loc.center.x, currentObjective->loc.center.y, 0);
    glBegin(GL_LINES);
    glColor4f(RED, 1.0);
    glVertex2f(-5, -5);
    glVertex2f(5, 5);
    glVertex2f(-5, 5);
    glVertex2f(5, -5);
    glEnd();
    glPopMatrix();
  }
}

void initialize_robot(AbstractRobot * bot, const char * mapPath, const char *botPath,
                                        const char * sensLibPath, bool arduinoMotor = true ) {
  SensorFactory * sensors = NULL;
  MotorFactory * motors = NULL;
  Arduino * arduino = new Arduino();
  
  if( arduinoMotor ) {
    motors = new ArduinoMotorFactory(arduino);
  }
  //else {
  //  motors = new MaestroMotorFactory(maestro);
  //}
  
  //sensors = new ArduinoSensorFactory(sensLibPath);
  
  sim::Map * map = sim::read_map(mapPath);
  MCL * mcl = new MCL();
  mcl->initialize(map->start, 10, *map);
  delete map;
  map = NULL;
  bot->addSlam(mcl);
  
  read_robot(bot, botPath, sensors, motors);
  
  delete motors;
  //delete sensors;
}
