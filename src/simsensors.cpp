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
  Ray absPos = world->simBot.position.transformRayToAbsolute(relPos);
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
  vec2 realPoint(specs.maxRange,0);
  const std::vector<sim::Wall>& walls = world->map->walls;
  for( unsigned int i = 0; i < walls.size(); i++ ) {
    float curDist = specs.maxRange;
    const Wall& curWall = walls[i];
    vec2 curPoint(specs.maxRange, 0);
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
      if( minDist(absPos.transformRayToAbsolute(dispRay)) < curDist ) {
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
  if( robot::SonarRobot * bot = dynamic_cast<robot::SonarRobot*> (world->bot) ) {
    bot->realPoints.push_back(absPos.transformVecToAbsolute(realPoint));
  }
  return dist;
}

vec2 sim::Encoder::getLastAbsolutePosition() {
  vec2 absPos = world->simBot.lastPosition.transformVecToAbsolute(relPos);
  return absPos;
}

vec2 sim::Encoder::getAbsolutePosition() {
  vec2 absPos = world->simBot.position.transformVecToAbsolute(relPos);
  return absPos;
}

long sim::Encoder::getCount() {
  vec2 lastPos = getLastAbsolutePosition();
  vec2 curPos = getAbsolutePosition();
  // first approximation, just go along the line
  vec2 disp = curPos - lastPos;
  double dist = (curPos-lastPos).mag();
  lastPos = curPos;
  // the vex encoders can tell which direction they are turning
  if( disp.dot(world->simBot.position.dir()) >= 0 ) {
    totalDist += dist;
  }
  else {
    totalDist -= dist;
  }
  count = totalDist/tickDist;
  return count;
}

robot::RangeSensor * sim::SensorFactory::allocRange(const robot::RangeSpecs& specs) {
  return new Ultrasonic(world, specs);
}

robot::Encoder * sim::SensorFactory::allocEncoder(float tickDist) {
  return new Encoder(world, tickDist);
}
