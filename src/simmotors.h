/* -*- C++ -*-
 *  simmotors.h
 *  sim
 */

#include "robot.h"
#include "simulation.h"

#ifndef __SIM_MOTORS_H__
#define __SIM_MOTORS_H__

namespace sim {
  
  // We could make this a lot more sophisticated
  class MotorControl : public robot::MotorControl {
    sim::Robot * bot;
    public:
    MotorControl(sim::Simulation * w) : bot(&(w->simBot)) {}
    virtual ~MotorControl() {}
    
    virtual void setVelocity(float velocity);
    virtual void setAngularVelocity(float angVel);
    
  };
  
  class MotorFactory : public robot::MotorFactory {
    sim::Simulation * world;
    public:
    MotorFactory(sim::Simulation* w = NULL) : world(w) {}
    
    void setWorld(sim::Simulation * w) {
      world = w;
    }
    
    virtual robot::MotorControl* newMotors();
  };
  
} // namespace sim

#endif // __SIM_MOTORS_H__
