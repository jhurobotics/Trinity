/*
 *  simsensors.cpp
 *  sim
 */

#include "simsensors.h"
#include "math.h"
#include <cstdlib>
#include <climits>
#include <iostream>
#include <fstream>
#include <string>
#include "geometryio.h"
using namespace sim;
using namespace math;

Ray sim::Ultrasonic::getAbsolutePosition() {
  Ray absPos = relPos;
  mat2 worldRotation = world->bot.position.dir().getRotationMatrix();
  absPos.transform(worldRotation);
  absPos += world->bot.position.origin();
  return absPos;
}

float sim::Ultrasonic::getValue() {
  Ray absPos = getAbsolutePosition();
  static const vec2 origin(0.0, 0.0);
  vec2 upperRay(1.0, specs.tanOfWidth);
  upperRay.normalize();
  vec2 lowerRay(1.0, -specs.tanOfWidth);
  lowerRay.normalize();
  
  // In the perfect simulation model,
  // find the nearest distance
  float dist = specs.maxRange;
  const std::vector<sim::Wall>& walls = world->map->walls;
  for( unsigned int i = 0; i < walls.size(); i++ ) {
    float curDist = specs.maxRange;
    const Wall& curWall = walls[i];
    Wall transWall(absPos.transformToLocal(curWall[0]), absPos.transformToLocal(curWall[1]));
    // the wall must be in front of the sensor
    if( transWall[0].x < 0.0 && transWall[1].x < 0.0 ) {
      continue;
    }

    // see if this wall is in the cone of the pulse
    float firstTan = transWall[0].y/transWall[0].x;
    float secondTan = transWall[1].y/transWall[1].x;
    if( ( firstTan >  specs.tanOfWidth && secondTan <  specs.tanOfWidth ) ||
        ( firstTan < -specs.tanOfWidth && secondTan > -specs.tanOfWidth ) ||
        ( fabsf(firstTan) < specs.tanOfWidth || fabsf(secondTan) < specs.tanOfWidth )
      ) {
      float positionOnWall;
      vec2 disp = pointToSeg(origin, transWall, &positionOnWall);
      if( disp.y / disp.x > specs.tanOfWidth ) {
        curDist = distance(Ray(vec2(0.0, 0.0), upperRay), transWall);
      }
      else if( disp.y / disp.x < -specs.tanOfWidth ) {
        curDist = distance(Ray(vec2(0.0, 0.0), lowerRay), transWall);
      }
      else {
        curDist = disp.mag();
      }
    }

    if( curDist > 0 && curDist < dist ) {
      dist = curDist;
    }
  }
  
  // Default value of dist = maxRange, and we only save smaller (nonnegative)
  // ones, so don't need to check for longer values
  /*if( dist > maxRange ) {
    dist = maxRange;
  }
  else*/
  if( dist < specs.minRange ) {
    // If the wall is too close, then the sensor starts listening 
    // after the pulse comes back, so it never gets the pulse, so max
    dist = specs.maxRange;
  }
  
  // Add noise
  dist *= randFloat(specs.error, 1.0);
  
  // log this data point for visualization
  // log( absPos.origin() + (dist * absPos.dir()) );
  
  return dist;
}

std::ifstream * sim::SensorFactory::openSensorFile(const char * name) {
  return new std::ifstream ((libPath + "/" + name).c_str());
}

robot::Sensor * sim::SensorFactory::newSensorWithName(const char* name, robot::SensorKind * pKind) {
  robot::Sensor * result = NULL;
  std::ifstream *in = openSensorFile(name);
  std::ifstream & input = *in;
  
  try {
    robot::SensorKind kind = robot::UNKNOWN;
    std::string astring;
    while( input ) {
      input >> astring;
      if( astring[0] == '#' ) {
        input.ignore(LONG_MAX, '\n');
      }
      else if( astring == "kind" ) {
        input >> astring;
        if( astring == "RANGE" ) {
          kind = robot::RANGE;
        }
        else {
          std::cerr << "Unrecognized type in sensor " << name << ": " << astring << "\n";
          throw 1;
        }
        if( pKind ) {
          *pKind = kind;
        }
        break;
      }
    }
    
    switch( kind ) {
      case robot::RANGE:
        result = newRangeSensor(input);
      default:
        throw 2;
    }
  }
  catch(...) {
  }
  input.close();
  delete in;
  return result;
}

robot::RangeSensor * sim::SensorFactory::rangeSensor(const char * name) throw(WrongSensorKind) {
  std::ifstream * in = openSensorFile(name);
  std::ifstream& input = *in;
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
      break;
    }
  }  
  robot::RangeSensor * result = newRangeSensor(input);
  input.close();
  delete in;
  return result;
}

robot::RangeSensor * sim::SensorFactory::newRangeSensor(std::istream& input) {
  RangeSpecs specs;
  std::string astring;
  while( input ) {
    input >> astring;
    if( astring[0] == '#' ) {
      input.ignore(LONG_MAX, '\n');
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
  return new Ultrasonic(world, specs);
}
