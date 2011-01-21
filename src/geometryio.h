/*
 *  geometryio.h
 *  sim
 */

#ifndef __GEOMETRY_IO__
#define __GEOMETRY_IO__

#include <iostream>
#include "geometry.h"

static inline std::istream& operator>>(std::istream& input, math::vec2& vec) {
  return input >> vec.x >> vec.y;
}

static inline std::istream& operator>>(std::istream& input, math::Segment& seg) {
  math::vec2 ends[2];
  input >> ends[0] >> ends[1];
  seg = math::Segment(ends[0], ends[1]);
  return input;
}

static inline std::istream& operator>>(std::istream& input, math::Ray& ray) {
  math::vec2 vec;
  input >> vec;
  ray.setOrigin(vec);
  input >> vec;
  ray.setDir(vec);
  return input;
}

static inline std::istream& operator>>(std::istream& input, math::Circle& c) {
  float radius;
  input >> c.center >> radius;
  c.setRadius(radius);
  return input;
}

static inline std::ostream& operator<<(std::ostream& output, const math::vec2& vec) {
  return output << "(" << vec.x << ", " << vec.y << ")";
}

static inline std::ostream& operator<<(std::ostream& output, const math::Segment& wall) {
  return output << wall[0] << ", " << wall[1];
}

static inline std::ostream& operator<<(std::ostream& output, const math::Ray& ray) {
  return output << ray.origin() << ", " << ray.dir();
}

static inline std::ostream& operator<<(std::ostream& output, const math::mat2& mat) {
  output << "{ {" << mat.data[0][0] << ", " << mat.data[0][1] << "}, {";
  return output << mat.data[1][0] << ", " << mat.data[1][1] << "} }";
}

#endif // __GEOMETRY_IO__
