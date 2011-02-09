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
  
  struct Measurements {
  };
  
  struct MeasurementMap {
  };
  
  class SLAM { 
  public:
    virtual Pose getPose() = 0;
    virtual void draw() { }
  }; // class SLAM
  
  class MCL : public SLAM {
    protected:
    typedef std::vector<Pose> belief_t;
    typedef std::vector< std::pair<Pose, float> > weighted_belief_t;
    
    float odometryNoise[4]; // robot specific parameters
    
    belief_t bels[2];
    unsigned char cur_bel;
    MeasurementMap map;
   
    Pose sample_motion_model_odometry(const Odometry& u_t, const Pose& lastPose);
    float sample_measurement_model(const Measurements& z, const Pose& x);
    
    void low_variance_sampler(const weighted_belief_t & input, float total,
                         belief_t * output);
    void mcl( const belief_t& last_bel, const Odometry& u, const Measurements& z,
              belief_t * new_bel);
    Pose getAverage(const belief_t & bel);
    
    public:
    explicit MCL() {
      cur_bel = 0;
    }
    void initialize(Pose cur, float range);
    virtual Pose getPose();
    virtual void draw();
  };
  
} // namespace robot

#endif // __SLAM_H__