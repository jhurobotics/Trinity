/* -*-C++-*-
 *  simsensors.h
 */

#include "robot.h"
#include "simulation.h"
#include "map.h"
#include <string>

#ifndef __SIMSENSORS_H__
#define __SIMSENSORS_H__

namespace sim {

  class Ultrasonic : public robot::RangeSensor {
    protected:
    Simulation * world;
    
    public:
    Ultrasonic(sim::Simulation * w, const robot::RangeSpecs& s)
      : robot::RangeSensor(s), world(w)
    {}
            
    protected:
    // Find my actual position in the house
    math::Ray getAbsolutePosition();
    float minDist(const math::Ray& ray);
    
    public:
    // Generate a noisy data point
    virtual float getValue();
  }; // class Ultrasonic
  
  class Encoder : public robot::Encoder {
    protected:
    long count;
    Simulation * world;
    math::vec2 getLastAbsolutePosition();
    math::vec2 getAbsolutePosition();
    public:
    Encoder(sim::Simulation * w, float td)
      : robot::Encoder(td), count(0), world(w)
    {}
    
    virtual long getCount();
  };

  class SensorFactory : public robot::SensorFactory {
    std::string libPath;
    sim::Simulation * world;
    public:
    SensorFactory(const std::string& lib, sim::Simulation * w)
      : libPath(lib), world(w) {}
    virtual ~SensorFactory() {}
    
    virtual robot::RangeSensor * rangeSensor(const std::string& name) throw(WrongSensorKind);
    virtual robot::Encoder * encoder(const std::string& name) throw(WrongSensorKind);
  };
} // namespace sim

#endif // __SIMSENSORS_H__
