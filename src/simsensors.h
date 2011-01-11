/*
 *  simsensors.h
 *  sim
 */

#ifndef __SIMSENSORS_H__
#define __SIMSENSORS_H__

#include "robot.h"
#include "simulation.h"
#include "map.h"
#include <fstream>
#include <string>

namespace sim {
    
  class Ultrasonic : public robot::RangeSensor {
    Simulation * world;
    
    public:
    Ultrasonic(sim::Simulation * w, const robot::RangeSpecs& s)
      : robot::RangeSensor(s), world(w)
    {}
            
    protected:
    // Find my actual position in the house
    math::Ray getAbsolutePosition();
    
    public:
    // Generate a noisy data point
    virtual float getValue();
  }; // class Ultrasonic

  class SensorFactory : public robot::SensorFactory {
    std::string libPath;
    sim::Simulation * world;
    public:
    SensorFactory(const std::string& lib, sim::Simulation * w)
      : libPath(lib), world(w) {}
    virtual ~SensorFactory() {}
    
    virtual robot::RangeSensor * rangeSensor(const std::string& name) throw(WrongSensorKind);
  };
} // namespace sim

#endif // __SIMSENSORS_H__
