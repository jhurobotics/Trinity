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

// angle of incidence
#define REFLECT_ANGLE 20.0*M_PI/180.0
static const float maxDotProd = cos(M_PI/2.0 - REFLECT_ANGLE);

Ray sim::Ultrasonic::getAbsolutePosition() {
  Ray absPos = world->bot.position.transformToAbsolute(relPos);
  return absPos;
}

float sim::Ultrasonic::minDist(const Ray& ray) {
  Ray absPos = getAbsolutePosition();
  
  // In the perfect simulation model,
  // find the nearest distance
  float dist = -10;
  const std::vector<sim::Wall>& walls = world->map->walls;
  for( unsigned int i = 0; i < walls.size(); i++ ) {
    const Wall& curWall = walls[i];
    float curDist = distance(ray, curWall);
    if( dist < 0 || (curDist > 0 && curDist < dist) ) {
      dist = curDist;
    }
  }
  return dist;
}

float sim::Ultrasonic::getValue() {
  Ray absPos = getAbsolutePosition();
  static const vec2 origin(0.0, 0.0);
  Ray upperRay(vec2(0, 0), vec2(1.0,  specs.tanOfWidth).normalize());
  Ray lowerRay(vec2(0, 0), vec2(1.0, -specs.tanOfWidth).normalize());
  
  // In the perfect simulation model,
  // find the nearest distance
  float dist = specs.maxRange;
  vec2 realPoint(-10,0);
  const std::vector<sim::Wall>& walls = world->map->walls;
  for( unsigned int i = 0; i < walls.size(); i++ ) {
    float curDist = specs.maxRange;
    const Wall& curWall = walls[i];
    vec2 curPoint(-10, 0);
    Wall transWall(absPos.transformToLocal(curWall[0]), absPos.transformToLocal(curWall[1]));
    // the wall must be in front of the sensor
    if( transWall[0].x < 0.0 && transWall[1].x < 0.0 ) {
      continue;
    }

    // see if this wall is in the cone of the pulse
    float firstTan = transWall[0].y/transWall[0].x;
    float secondTan = transWall[1].y/transWall[1].x;
    // either the wall crosses the boundary of the pulse, or both ends must be inside
    if( ( fabsf(firstTan) < specs.tanOfWidth || fabsf(secondTan) < specs.tanOfWidth ) ||
        ( distance(upperRay, transWall) > 0 || distance(lowerRay, transWall) > 0 )
      ) {
      float positionOnWall;
      vec2 disp = pointToSeg(origin, transWall, &positionOnWall);
      Ray dispRay(origin, disp);
      if( disp.y / disp.x > specs.tanOfWidth ) {
        curDist = distance(upperRay, transWall);
      }
      else if( disp.y / disp.x < -specs.tanOfWidth ) {
        curDist = distance(lowerRay, transWall);
      }
      else {
        curDist = disp.mag();
      }
      curPoint = disp;
      // if the angle of incidence is low enough, then the beam reflects away off of the wall, so discard the signal
      float cosOfAngle = disp.dot(transWall.direction()) / disp.mag() / transWall.direction().mag();
      if( fabsf(cosOfAngle) > maxDotProd ) {
        continue;
        // do raytracing?
      }

      // is this actually the first thing that gets hit?
      if( minDist(absPos.transformToAbsolute(dispRay)) < curDist ) {
        continue;
      }
    }
    else {
      continue;
    }


    if( curDist > 0 && curDist < dist ) {
      dist = curDist;
      realPoint = curPoint;
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
  dynamic_cast<robot::Robot*>(world->bot.bot)->realPoints.push_back(absPos.transformToAbsolute(realPoint));
  return dist;
}

robot::RangeSensor * sim::SensorFactory::rangeSensor(const std::string& name) throw(WrongSensorKind) {
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
  return new Ultrasonic(world, rangeSensors[name]);
}
