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
#include "CandleScan.h"
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

SonarRobot::SonarRobot() throw() : curMode(INIT), curScanMode(START),
                                   currentObjective(NULL), rangeFinders(),
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
  
  switch( curMode ) {
    case INIT:
      // Only move once the SLAM module has settled
      if( !slammer->settled() ) {
        break;
      }
      else {
        static const vec2 NORTH( 0, 1);
        static const vec2 EAST ( 1, 0);
        curMode = HALLWAY;
        vec2 dir = position.dir();
        float d;
        if( abs(d = dir.dot(NORTH)) > 0.5 ) {
          if( d > 0 ) {
            graph->turn(N);
          }
          else {
            graph->turn(S);
          }
        }
        else {
          if( dir.dot(EAST) > 0 ) {
            graph->turn(E);
          }
          else {
            graph->turn(W);
          }
        }
      }
    case HALLWAY:
      hallway();
      break;
    case SCAN:
      scan();
      break;
    default:
      break;
  }
}

#define MOVE_SPEED 20
#define TURN_SPEED M_PI / 8.0

void SonarRobot::hallway() throw() {
  static bool moving = false;
  static int cur_dir = 0;
  if( !currentObjective ) {
    // get an objective!
    cur_dir = graph->traverse(&currentObjective);
    graph->turn(cur_dir);
    graph->move();
  }
  
  // am I at the objective?
  if( currentObjective->position.contains(position.origin()) ) {
    control->setVelocity(0);
    control->setAngularVelocity(0);
    moving = false;
    if( !currentObjective->checked ) {
      if( currentObjective->room ) {
        curMode = SCAN;
      }
      else {
        // we're in a hallway, and we need to do something about it
        // we need to check out something in the hallway
        // this is probably where one of the uncertain doors is
      }
    }
    else {
      int dir = graph->traverse();
      currentObjective = graph->cur->getnode(dir);
      int destDir = (dir + cur_dir) % 4;
      graph->turn(destDir);
      graph->move();
    }
  }
  else { // get there
    vec2 disp = currentObjective->position.center - position.origin();
      
    disp.normalize();
    float curDir = position.angle();
    if( disp.dot(position.dir()) > 0.99 ) {
      control->setVelocity(MOVE_SPEED);
      control->setAngularVelocity(0);
      moving = true;
    }
    else {
      if( moving )
        control->setVelocity(0);
      float delta = atan2(disp.y, disp.x) - curDir;
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

// turn 180 degrees to the left in 8 seconds
#define SCAN_SPEED M_PI / 8

void SonarRobot::scan() throw() {
  static float startAngle;
  float diff;
  switch( curScanMode ) {
    case START:
      startAngle = slammer->getPose().angle();
      control->setVelocity(0);
      control->setAngularVelocity( -SCAN_SPEED);
      curScanMode = TURN_RIGHT;
    case TURN_RIGHT:
      diff = startAngle - slammer->getPose().angle();
      if( diff < 0 ) {
        diff += 2*M_PI;
      }
      if( diff < M_PI / 2 ) {
        break;
      }
      else {
        // start the left scan
        curScanMode = SCAN_LEFT;
        control->setAngularVelocity( SCAN_SPEED );
      }
    case SCAN_LEFT:
      // check to see if we saw anthing
      // if( see something ) {
      //  say something;
      //  curMode = VERIFY_READING;
      //  motors->setAngularVelocity(0);
      //  break;
      // }
      diff = slammer->getPose().angle() - startAngle;
      if( diff < 0 ) {
        diff += 2*M_PI;
      }
      if( diff < M_PI / 2 || diff > 3 * M_PI / 2) {
        break;
      }
      else {
        curScanMode = FINISHED;
        control->setAngularVelocity(0);
      }
    case FINISHED:
      // return to hallway mode
      curScanMode = START;
      curMode = HALLWAY;
      break;
    case VERIFY_READING:
      // No-op for now
      control->setAngularVelocity(SCAN_SPEED);
      curScanMode = SCAN_LEFT;
      break;
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
#define THOUGHT 1.0, 1.0, 0.5

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
