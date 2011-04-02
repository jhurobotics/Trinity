/*
 *  MeasurementModel.cpp
 */

#include "slam.h"
#include "../map.h"
#include "../geometry.h"
#include "../robot.h"
#include <cmath>
using namespace robot;
using namespace math;

float MCL::minDist(const Ray& ray, const Pose& x) {
  Ray absPos = x.transformRayToAbsolute(ray);
  
  // In the perfect simulation model,
  // find the nearest distance
  float dist = -10;
  const std::vector<sim::Wall>& walls = map->walls;
  for( unsigned int i = 0; i < walls.size(); i++ ) {
    const sim::Wall& curWall = walls[i];
    float curDist = distance(absPos, curWall);
    if( dist < 0 || (curDist > 0 && curDist < dist) ) {
      dist = curDist;
    }
  }
  return dist;
}

float MCL::sample_measurement_model(const Pose& x) {
  float weight = 1.0;
  
  typedef std::set<RangeSensor*>::iterator RangeIter_t;
  RangeIter_t end = rangeFinders.end();
  for( RangeIter_t iter = rangeFinders.begin(); iter != end; iter++) {
    float real = (*iter)->getValue();
    float fake = minDist((*iter)->relPos, x);
    if( std::abs((real - fake)) < fake * (*iter)->specs.error ) {
      weight *= 0.9;
    }
    else {
      weight *= 0.1;
    }
  }
  return weight;
}
