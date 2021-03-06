/*
 *  geometry.cpp
 *  sim
 */
#include "geometry.h"
using namespace math;

mat2 math::getRotationMatrix(float angle) throw() {
  mat2 result;
  result.data[0][0] = result.data[1][1] = cos(angle);
  result.data[1][0] = sin(angle);
  result.data[0][1] = - result.data[1][0];
  return result;
}

float math::distance(const Ray& ray, const Segment& wall) throw() {
  vec2 b = wall.direction();
  
  float denom = b.cross(ray.dir());
  if( fabsf(denom) < 0.01 ) {
    return -10.0f;
  }
  
  float t = ( ray.dir().cross(wall.start) + ray.origin().cross(ray.dir()) ) / denom;
  if( t < 0.0 || t > 1.0 ) {
    return -10.0f;
  }
  
  float s = -( b.cross(ray.origin()) + wall[0].cross(b) ) / denom;
  return s;
}

vec2 math::pointToSeg(const vec2& c, const Segment& wall, float *param) throw() {
  vec2 b = wall.direction();
  float t = b.dot(c - wall.start) / b.mag_sq();
  if( t > 1.0 ) {
    t = 1.0;
  }
  else if( t < 0.0 ) {
    t = 0.0;
  }

  if( param ) {
    *param = t;
  }
  return wall.start + t*b - c;
}

bool math::intersect(const Circle& c, const Segment& wall) throw() {
  float t;
  vec2 disp = pointToSeg(c.center, wall, &t);
  if( disp.mag_sq() <= c.radius()*c.radius() ) {
    return true;
  }
  else {
    return false;
  }
}
