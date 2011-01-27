/*
 *  math.h
 *  sim
 */

#include <climits>
#include <cstdlib>

#ifndef __ROBOT_MATH_H__ // __MATH_H__ is taken by the real math.h
#define __ROBOT_MATH_H__

namespace math {
  
  static inline float randFloat(float mag, float center = 0.0) {
    return (static_cast<float>(random())/static_cast<float>(INT_MAX)
            * 2.0 - 1.0)* mag + center;
  }

} // namespace math

#endif // __ROBOT_MATH_H__
