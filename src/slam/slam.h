/*
 *  slam.h
 *  sim
 */

#ifndef __SLAM_H__
#define __SLAM_H__

#include <vector>
#include "math.h"
#include "../geometry.h"

namespace robot {
  typedef math::Ray Pose;
  
  struct Odometry {
    math::Ray prev;
    math::Ray next;
  };
  
  extern float odometryNoise[4]; // robot specific parameters
  
  Pose sample_motion_model_odometry(const Odometry& u_t, const Pose& lastPose);
  
  struct Measurements {
  };
  
  struct MeasurementMap {
  };
  
  float sample_measurement_model(const Measurements& z, const Pose& x, const MeasurementMap& m);
  
  typedef std::vector<Pose> belief_t;
  typedef std::vector< std::pair<Pose, float> > weighted_belief_t;
  void mcl( const belief_t& last_bel, const Odometry& u, const Measurements& z,
           const MeasurementMap map, belief_t * new_bel);
  Pose getAverage(const robot::belief_t & bel);
} // namespace robot

#endif // __SLAM_H__