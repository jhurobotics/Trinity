/*
 *  slam.h
 *  sim
 */

#include "geometry.h"
#include "simulation.h"
#include "slam.h"

#ifndef __SIM_SLAM_H__
#define __SIM_SLAM_H__

namespace robot {
    
  class SimSLAM : public SLAM {
    sim::Robot * bot;
    public:
    SimSLAM(sim::Robot * r) : SLAM(), bot(r) {}
    
    virtual math::Ray getPose() {
      return bot->position;
    }
  }; // class SimSLAM
  
} // namespace robot

#endif // __SIM_SLAM_H__