/*
 *  slam.h
 *  sim
 */

#include "geometry.h"
#include "simulation.h"

#ifndef __SLAM_H__
#define __SLAM_H__

namespace robot {
  
  class SLAM { 
    public:
    virtual math::Ray getPose() = 0;
  }; // class SLAM
  
  class SimSLAM : public SLAM {
    sim::Robot * bot;
    public:
    SimSLAM(sim::Robot * r) : SLAM(), bot(r) {}
    
    virtual math::Ray getPose() {
      return bot->position;
    }
  }; // class SimSLAM
  
} // namespace robot

#endif // __SLAM_H__