/*
 *  slam.h
 *  sim
 */

#ifndef __SLAM_H__
#define __SLAM_H__

#include "../geometry.h"

namespace robot {
  typedef math::Ray Pose;
  
  struct Odometry {
    math::Ray prev;
    math::Ray next;
  };
  
  float odometryNoise[4]; // robot specific parameters
} // namespace robot

#endif // __SLAM_H__