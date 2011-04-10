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
#include "speedTest.h"

#define MAX_EXTINGUISH_TIME 8000

using namespace robot;
using namespace math;

AbstractRobot * robot::new_robot(robot::Implementation imp) throw(BadRobotImplementation) {
  switch( imp ) {
  case SONAR:
    return new SonarRobot;
  case SPEEDTEST:
    return new SpeedTest;
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
  std::string pathStr = path;
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
    else if( astring == "uvsensor" ) {
      UVSensor * uv = sensors->uvsensor();
      uv->relPos = read_position(input, sensorPositions);
      input >> uv->id;
      bot->addUV(uv);
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
      Graph * g = new Graph();
      std::string graphPath = pathStr.substr(0, pathStr.find_last_of('/')+1) + astring;
      read_graph(g, graphPath.c_str());
      bot->addGraph(g);
    }
    else if( astring == "map" ) {
      std::string name;
      input >> name;
      input >> astring;
      std::string mapPath = pathStr.substr(0, pathStr.find_last_of('/')+1) + astring;
      sim::Map * map = sim::read_map(mapPath.c_str());
      bot->addMap(map, name);
    }
  }
  
  input.close();
}

SonarRobot::SonarRobot() throw() :  curMode(INIT), curDecision(ROOM1A),
                                    decidingEye(NULL), curScanMode(START),
                                    currentObjective(NULL), rangeFinders(),
                                    encoders(), uvtron(NULL), control(), edges(),
                                    path(), uvBaselineCount(0), uvBaseline(0),
                                    position()
{
  mapVector.reserve(4);
  currentMapIndex = 0;
  slammer = new MCL();
  mapVector[0] = mapVector[1] = mapVector[2] = mapVector[3] = NULL;
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
        uvBaseline += uvtron->getValue();
        uvBaselineCount++;
        break;
      }
      uvBaseline /= uvBaselineCount;
    case HALLWAY:
      hallway();
      break;
    case HALLWAY_CHECK:
      hallwayCheck();
      break;
    case SCAN:
      scan();
      break;
		case EXTINGUISH:
      extinguish();
      break;
		case GO_HOME:
      goHome();
      break;
    default:
      break;
  }
}

#define MOVE_SPEED 20
#define TURN_SPEED M_PI / 8.0

void SonarRobot::hallway() throw() {
  static bool moving = false;
  if( !currentObjective ) {
    // get an objective!
    try {
      currentObjective = graph->getObjective(position.origin());
    }
    catch( Graph::AllCheckedException e) {
      halt();
      return;
    }
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
        curMode = HALLWAY_CHECK;
        if( currentObjective->name == "ROOM1_ENTER" ) {
          curDecision = ROOM1A;
        }
        else if( currentObjective->name == "ROOM2_ENTER" ) {
          curDecision = ROOM1B;
        }
        else if( currentObjective->name == "ROOM4A_ENTER" ) {
          curDecision = ROOM4A;
        }
        else if( currentObjective->name == "ROOM4B_ENTER" ) {
          curDecision = ROOM4B;
        }
        hallwayCheck();
      }
    }
    else {
      currentObjective = graph->getObjective(position.origin());
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

// the robot will take this many measurements
// before it decides that a wall is or is not there
#define DECISION_THRESHOLD  4
// If the measurements average less than 20 cm, then assume
// that there is a wall in that direction
#define WALL_CUTOFF         30.0

void SonarRobot::hallwayCheck() throw() {
  if( ! decidingEye ) {
    switch( curDecision ) {
      case ROOM1A:
      case ROOM1B:
        doorDir = vec2(-1, 0);
        break;
      case ROOM4A:
        doorDir = vec2(0, 1);
        break;
      case ROOM4B:
        doorDir = vec2(0, -1);
        break;
    }
    // find the sensor lookiing in the given absolute direction
    rangeIter_t end = rangeFinders.end();
    float bestDot = -2.0;
    for( rangeIter_t iter = rangeFinders.begin(); iter != end && !decidingEye; iter++ ) {
      float dot = position.transformRayToAbsolute((*iter)->relPos).dir().dot(doorDir);
      if( dot > bestDot ) {
        bestDot = dot;
        decidingEye = *iter;
      }
    }
    totalMeasure = lastMeasurement = 0;
    decisionCount = 0;
  }
  
  
  vec2 lookDir = position.transformRayToAbsolute(decidingEye->relPos).dir();
  float cross = lookDir.cross(doorDir);
  if( cross > 0.2 ) {
    control->setAngularVelocity(TURN_SPEED);
    return;
  }
  else if( cross < -0.2 ) {
    control->setAngularVelocity(-TURN_SPEED);
    return;
  }
  else {
    control->setAngularVelocity(0);
  }
  
  float measure = decidingEye->getValue();
  
  if( measure != lastMeasurement ) {
    lastMeasurement = measure;
    totalMeasure += lastMeasurement;
    decisionCount ++;
    if( decisionCount >= DECISION_THRESHOLD ) {
      float avgDist = totalMeasure / static_cast<float>(decisionCount);
      Node * doors[2];
      Node * openNode;
      Node * closeNode;
      switch( curDecision ) {
        case ROOM1A:
          doors[0] = graph->nodeByName["ROOM1_ENTER"];
          doors[1] = graph->nodeByName["ROOM2_ENTER"];
          if( avgDist > WALL_CUTOFF ) {
            openNode = graph->nodeByName["ROOM1A"];
            closeNode = graph->nodeByName["ROOM1B"];
            currentMapIndex |= map_1A;	   
          }
          else {
            openNode = graph->nodeByName["ROOM1B"];
            closeNode = graph->nodeByName["ROOM1A"];
            currentMapIndex |= map_1B;
          }
          break;
        case ROOM1B:
          doors[0] = graph->nodeByName["ROOM1_ENTER"];
          doors[1] = graph->nodeByName["ROOM2_ENTER"];
          if( avgDist > WALL_CUTOFF ) {
            openNode = graph->nodeByName["ROOM1B"];
            closeNode = graph->nodeByName["ROOM1A"];
            currentMapIndex |= map_1B;
          }
          else {
            openNode = graph->nodeByName["ROOM1A"];
            closeNode = graph->nodeByName["ROOM1B"];
            currentMapIndex |= map_1A;
          }
          break;
        case ROOM4A:
          doors[0] = graph->nodeByName["ROOM4A_ENTER"];
          doors[1] = graph->nodeByName["ROOM4B_ENTER"];
          if( avgDist > WALL_CUTOFF ) {
            openNode = graph->nodeByName["ROOM4A"];
            closeNode = graph->nodeByName["ROOM4B"];
            currentMapIndex |= map_4A;
          }
          else {
            openNode = graph->nodeByName["ROOM4B"];
            closeNode = graph->nodeByName["ROOM4A"];
            currentMapIndex |= map_4B;
          }
          break;
        case ROOM4B:
          doors[0] = graph->nodeByName["ROOM4A_ENTER"];
          doors[1] = graph->nodeByName["ROOM4B_ENTER"];
          if( avgDist > WALL_CUTOFF ) {
            openNode = graph->nodeByName["ROOM4B"];
            closeNode = graph->nodeByName["ROOM4A"];
            currentMapIndex |= map_4B;
          }
          else {
            openNode = graph->nodeByName["ROOM4A"];
            closeNode = graph->nodeByName["ROOM4B"];
            currentMapIndex |= map_4A;
          }
          break;
      }
      doors[0]->checked = doors[1]->checked = true;
      closeNode->checked = true;
      slammer->setMap(mapVector[currentMapIndex]);
      decidingEye = NULL;
      curMode = HALLWAY;
      currentObjective = graph->getObjective(position.origin());
    }
  }
}

// turn 180 degrees to the left in 8 seconds
#define SCAN_SPEED M_PI / 8
#define UV_THRESHOLD   82
  // corresponds to 0.4 volts

void SonarRobot::scan() throw() {
  static vec2 targetDir;
  switch( curScanMode ) {
    case START:
      targetDir = slammer->getPose().dir();

			//Rotate Pi CW
      targetDir = vec2(-targetDir.x, -targetDir.y);
      control->setVelocity(0);
      control->setAngularVelocity( -SCAN_SPEED);
      curScanMode = TURN_RIGHT;
    case TURN_RIGHT:
      // check to see if we saw anthing
      if( (uvTotal = uvtron->getValue()) > uvBaseline + UV_THRESHOLD ) {
        lastTime = time();
        decisionCount = 1;
        lastScanMode = curScanMode;
        curScanMode = VERIFY_READING;
        control->setAngularVelocity(0);
        break;
      }

      if( position.dir().dot(targetDir) < 0.99 ) {
        break;
      }
      else {
        // start the left half scan
        curScanMode = SCAN_LEFT;
        control->setAngularVelocity( -SCAN_SPEED );
        targetDir *= -1;
      }
    case SCAN_LEFT:
      // check to see if we saw anthing
      if( (uvTotal = uvtron->getValue()) > uvBaseline + UV_THRESHOLD ) {
        lastTime = time();
        decisionCount = 1;
        lastScanMode = curScanMode;
        curScanMode = VERIFY_READING;
        control->setAngularVelocity(0);
        break;
      }
      
      if( position.dir().dot(targetDir) < 0.99 ) {
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
      currentObjective->checked = true;
      break;
    case VERIFY_READING:
      // take 3 more readings 0.05 seconds apart, takes 0.2s total
      timeval t = time();
      if( time_diff(lastTime, t) > 50000 ) {
        lastTime = t;
        uvTotal += uvtron->getValue();
        decisionCount++;
        
        if( decisionCount > 4 ) {
          if( uvTotal / decisionCount > UV_THRESHOLD + uvBaseline ) {
            curMode = EXTINGUISH;
          }
          else {
            control->setAngularVelocity(SCAN_SPEED);
            curScanMode = lastScanMode;
          }
        }
      }
      break;
  }
}

void SonarRobot::extinguish() throw() {
	if(false){ //light is on
		//slowly advanceRobot
		extinguishTimer = robot::time();
	}else if(time_diff(extinguishTimer, robot::time()) < MAX_EXTINGUISH_TIME){
			// keep blowing fan
	}else{
		curMode = GO_HOME;
	}
}

void SonarRobot::goHome() throw() {

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
#define THOUGHT 1.0, 0.0, 0.5

void SonarRobot::draw() {
#ifndef NO_GUI
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
#endif
}
