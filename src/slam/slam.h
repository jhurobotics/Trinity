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
  
  template<typename T>
  void low_variance_sampler(const std::vector< std::pair<T, float> > & input,
                            std::vector< T > * o) {
    assert(o);
    std::vector< std::pair<T, float> > & output = (*o);
    output.reserve(input.size());
    
    float r = math::randFloat(0.5, 0.5);
    float c = input[0].second;
    unsigned int i = 0;
    float M_inverse = 1.0 / ((float) input.size());
    for( unsigned int m = 0; m < input.size(); m++ ) {
      float u = r + m * M_inverse;
      while( u > c ) {
        i++;
        c += input[i].second;
      }
      output.push_back(input[i].first);
    }
  }
} // namespace robot

#endif // __SLAM_H__