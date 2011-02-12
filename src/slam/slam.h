/*
 *  slam.h
 *  sim
 */

#include <vector>
#include <set>
#include "math.h"
#include "../geometry.h"

#ifndef __SLAM_H__
#define __SLAM_H__

namespace robot {
  typedef math::Ray Pose;
  
#ifndef __ROBOT_H__
  class RangeSensor;
  class Encoder;
#endif
  
  struct Odometry {
    Pose prev;
    Pose next;
  };
  
  struct Measurements {
  };
  
  struct MeasurementMap {
  };
  
  class SLAM { 
  public:
    virtual Pose getPose() = 0;
    virtual void draw() { }
    virtual void addRangeSensor(RangeSensor * r) = 0;
    virtual void addEncoder(Encoder * r) = 0;
  }; // class SLAM
  
  class MCL : public SLAM {
    protected:
    std::set<RangeSensor*> rangeFinders;
    std::vector<Encoder*> encoders;
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
    virtual void addRangeSensor(RangeSensor * sensor) {
      rangeFinders.insert(sensor);
    }
    virtual void addEncoder(Encoder * encoder) {
      encoders.push_back(encoder);
    }
  };
  
} // namespace robot

#endif // __SLAM_H__