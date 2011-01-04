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
  
  struct RangeSpecs {
    // The maximum detectable range
    float maxRange;
    // The minimum detectable range
    float minRange;
    // The uncertainty in the measurement
    // this is proportional to the distance, i.e. +/- 1%
    float error;
  };    
  
  class Ultrasonic : public robot::RangeSensor {
    RangeSpecs specs;
    Simulation * world;
    
    public:
    Ultrasonic(sim::Simulation * w, const RangeSpecs& s)
      : world(w), specs(s)
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
    
    virtual robot::Sensor * newSensorWithName(const char * name,
                            robot::SensorKind* kind = NULL);
    virtual robot::RangeSensor * rangeSensor(const char * name) throw(WrongSensorKind);
    private:
    std::ifstream * openSensorFile(const char * name);
    robot::RangeSensor * newRangeSensor(std::istream& input);
  };
} // namespace sim

#endif // __SIMSENSORS_H__
