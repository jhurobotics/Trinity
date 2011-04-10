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
    double totalDist;
    long count;
    Simulation * world;
    math::vec2 getLastAbsolutePosition();
    math::vec2 getAbsolutePosition();
  public:
    Encoder(sim::Simulation * w, float td)
      : robot::Encoder(td), totalDist(0), count(0), world(w)
    {}
    
    virtual long getCount();
  };
  
  class UVSensor : public robot::UVSensor {
  public:
    virtual int getValue() {
      return 0;
    }
  };

  class SensorFactory : public robot::SensorFactory {
  protected:
    sim::Simulation * world;
    
  public:
    virtual robot::RangeSensor * allocRange(const robot::RangeSpecs& specs);
    virtual robot::Encoder * allocEncoder(float tickDist);
    
    SensorFactory(const std::string& lib, sim::Simulation * w)
    : robot::SensorFactory(lib), world(w) { }
    virtual ~SensorFactory() {}
    
    virtual robot::UVSensor * uvsensor() throw(WrongSensorKind) {
      return new UVSensor();
    }
  };
} // namespace sim

#endif // __SIMSENSORS_H__
