/*
 *  robot.cpp
 *  sim
 */

#include <climits>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>
#include "robot.h"
#include "geometryio.h"
#ifndef __APPLE__
#include <GL/gl.h>
#else
#include <OpenGL/gl.h>
#endif
#include "arduino.h"
#include "maestro.h"
#include "timers.h"
using namespace robot;
using namespace math;

AbstractRobot * robot::new_robot(robot::Implementation imp) throw(BadRobotImplementation) {
  switch( imp ) {
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
  if( maestro ) {
    delete maestro;
  }
}

void AbstractRobot::setArduino(Arduino * a) {
  if( arduino ) {
    delete arduino;
  }
  arduino = a;
}

void AbstractRobot::setMaestro(Maestro * m) {
  if( maestro ) {
    delete maestro;
  }
  maestro = m;
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

void robot::read_robot(AbstractRobot * bot, const char * path, SensorFactory * sensors) {
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
    else if( astring == "range" ) {
      input >> astring;
      RangeSensor * ranger = sensors->rangeSensor(astring.c_str());
      ranger->relPos = read_position(input, sensorPositions);
      input >> ranger->id;
      bot->addRangeSensor(ranger);
    }
    else if( astring == "encoder" ) {
      input >> astring;
      Encoder * encoder = sensors->encoder(astring.c_str());
      encoder->relPos = read_position(input, sensorPositions).origin();
      input >> encoder->id;
      bot->addEncoder(encoder);
    }
    else if( astring == "motor" ) {
      Motor m;
      input >> astring;
      m.relPos = read_position(input, sensorPositions);
      input >> m.id;
      input >> m.minSpeed >> m.maxSpeed;
      bot->addMotor(m);
    }
    else if( astring == "graph" ) {
      input >> astring;
      Graph * g = new Graph(0,0,0,0);
      std::string pathStr = path;
      std::string graphPath = pathStr.substr(0, pathStr.find_last_of('/')+1) + astring;
      math::read_graph(g, graphPath.c_str());
      bot->addGraph(g);
    }
  }
    
  input.close();
}

SonarRobot::SonarRobot() throw() : curMode(HALLWAY), currentObjective(NULL), rangeFinders(),
                                   control(), edges(), path(), position()
{
}

void robot::SonarRobot::act() throw() {
  // update the slammer
  struct timeval curTime = robot::time();
  if( robot::time_diff(lastSLAMTime, curTime) >= 100000 ) {
    position = slammer->getPose();
    lastSLAMTime = curTime;
  }
  else {
    position = slammer->increment(position);
  }
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
    currentObjective = graph->getObjective();
  }
  
  // am I at the objective?
  if( currentObjective->position.contains(position.origin()) ) {
    control->setVelocity(0);
    control->setAngularVelocity(0);
    moving = false;
    //if( !currentObjective->checked ) {
    //  curMode = SCAN;
    //}
    //else {
    //  currentObjective = graph->getObjective(position.origin());
    //}
  }
  else { // get there
    vec2 disp = currentObjective->position.center - position.origin();
      
    float dispDir = atan2(disp.y, disp.x);
    float curDir = position.angle();
    if( abs(std::fmod((double)dispDir - curDir, 2*M_PI)) < ANGLE_RES / (moving ? 1 : 2)
        || abs(std::fmod((double)dispDir - curDir, 2*M_PI)) > 2 * M_PI - (ANGLE_RES / (moving ? 1 : 2)) ) {
      control->setVelocity(MOVE_SPEED);
      control->setAngularVelocity(0);
      moving = true;
    }
    else {
      if( moving )
        control->setVelocity(0);
      float delta = dispDir - curDir;
      if( delta > M_PI ) {
        delta -= 2*M_PI;
      }
      else if( delta < -M_PI) {
        delta += 2*M_PI;
      }
      control->setAngularVelocity(TURN_SPEED * (delta > 0 ? 1 : -1 ));
    }
  }
}

RangeSensor * SensorFactory::rangeSensor(const std::string& name) throw(WrongSensorKind) {
  if( !rangeSensors.count(name) ) {
    robot::RangeSpecs specs;
    std::ifstream input((libPath + "/" + name).c_str());
    std::string astring;
    while( input ) {
      input >> astring;
      if( astring[0] == '#' ) {
        input.ignore(LONG_MAX, '\n');
      }
      else if( astring == "kind" ) {
        input >> astring;
        if( astring != "RANGE" ) {
          std::cerr << "Sensor named " << name << " is not a range sensor\n";
          throw WrongSensorKind();
        }
      }
      else if( astring == "maxRange" ) {
        input >> specs.maxRange;
      }
      else if( astring == "minRange" ) {
        input >> specs.minRange;
      }
      else if( astring == "error" ) {
        input >> specs.error;
      }
      else if( astring == "halfAngle" ) {
        float pulseWidth;
        input >> pulseWidth;
        specs.tanOfWidth = tanf(pulseWidth*M_PI/180.0f);
      }
    }  
    input.close();
    rangeSensors.insert(std::pair<std::string, robot::RangeSpecs>
                        (name, specs));
  }
  return allocRange(rangeSensors[name]);
}

Encoder * SensorFactory::encoder(const std::string& name) throw(WrongSensorKind) {
  if( !encoders.count(name) ) {
    unsigned long tickCount = -1;
    float wheelSize = -1;
    float tickDist = -1;
    std::ifstream input((libPath + "/" + name).c_str());
    std::string astring;
    while( input ) {
      input >> astring;
      if( astring[0] == '#' ) {
        input.ignore(LONG_MAX, '\n');
      }
      else if( astring == "kind" ) {
        input >> astring;
        if( astring != "ENCODER" ) {
          std::cerr << "Sensor named " << name << " is not an encoder\n";
          throw WrongSensorKind();
        }
      }
      else if( astring == "tickDist" ) {
        input >> tickDist;
      }
      else if( astring == "wheelSize" ) {
        input >> wheelSize;
      }
      else if( astring == "tickCount" ) {
        input >> tickCount;
      }
    }
    if( tickDist > 0 ) {
      encoders.insert(std::pair<std::string, float>(name, tickDist));
    }
    else {
      encoders.insert(std::pair<std::string, float>(name, wheelSize / tickCount));
    }
  }
  return allocEncoder(encoders[name]);
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
  graph->draw();
  
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
    glTranslatef(currentObjective->position.center.x, currentObjective->position.center.y, 0);
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
