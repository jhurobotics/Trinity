/* -*-C++-*-
 *  slam.h
 *  sim
 */

#include <vector>
#include <set>
#include "math.h"
#include "../geometry.h"
#include "../map.h"

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
  
  class SLAM { 
  public:
    virtual Pose getPose() = 0;
    virtual Pose increment(Pose curPose) = 0;
    virtual void draw() { }
    virtual void addRangeSensor(RangeSensor * r) = 0;
    virtual void addEncoder(Encoder * r) = 0;
    //! Can you trust the results of this module yet?
    //! The MCL needs to get rid of all the initial noise,
    //! but then gives very accurate results
    virtual bool settled() = 0;
    virtual void setMap(sim::Map* m) = 0;
    virtual bool isInitialized() = 0;
    virtual void initialize(Pose start, float range, sim::Map* m) = 0;
  }; // class SLAM
  
  class MCL : public SLAM {
    protected:
    bool inited;
    std::set<RangeSensor*> rangeFinders;
    std::vector<Encoder*> encoders;
    typedef std::vector<Pose> belief_t;
    typedef std::vector< std::pair<Pose, float> > weighted_belief_t;
    
    float odometryNoise[4]; // robot specific parameters
    
    belief_t bels[2];
    unsigned char cur_bel;
    sim::Map* map;
    Pose lastPose;
    long lastCount[2];
    long tmpCount[2];
    unsigned long cycleCount;
    
    float minDist(const math::Ray& ray, const Pose& x);
    Pose determineNext(Pose curPose, long encCount[2]);
    // This only moves IF the robot has moved > 1cm or turned > 1 degree
    Pose sample_motion_model_odometry(const Odometry& u_t, const Pose& lastPose);
    float sample_measurement_model(const Pose& x);
    
    void low_variance_sampler(const weighted_belief_t & input, float total, unsigned long count,
                         belief_t * output);
    void mcl( const belief_t& last_bel, const Odometry& u, belief_t * new_bel);
    Pose getAverage(const belief_t & bel);
    
    public:
    explicit MCL() {
      cur_bel = 0;
      odometryNoise[0] = odometryNoise[1] = 0.2;
      odometryNoise[2] = odometryNoise[3] = 0.4;
      cycleCount = 0;
      inited = false;
    }
    void initialize(Pose start, float range, sim::Map* m);
    bool isInitialized() {
      return inited;
    }
    virtual void setMap(sim::Map* m) {
      map = m;
    }
    void setMap(const sim::Map& m) {
      
    }
    virtual Pose getPose();
    virtual Pose increment(Pose curPose) {
      return determineNext(curPose, tmpCount);
    }
    virtual void draw();
    virtual void addRangeSensor(RangeSensor * sensor) {
      rangeFinders.insert(sensor);
    }
    virtual void addEncoder(Encoder * encoder) {
      encoders.push_back(encoder);
    }
    virtual bool settled() {
      return cycleCount > 5;
    }
  };
  
} // namespace robot

#endif // __SLAM_H__
