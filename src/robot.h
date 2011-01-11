/*
 *  robot.h
 *  sim
 */

#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <vector>
#include <map>
#include <string>
#include "geometry.h"

namespace robot {
  
  struct RangeSpecs {
    // The maximum detectable range
    float maxRange;
    // The minimum detectable range
    float minRange;
    // The uncertainty in the measurement
    // this is proportional to the distance, i.e. +/- 1%
    float error;
    // Tangent of half-of the angular width of the ultrasonic pulse in radians
    float tanOfWidth;
  };    
  
  class RangeSensor {
    public:
    RangeSpecs specs;
    // Position relative to the robot
    math::Ray relPos;
    RangeSensor(const RangeSpecs& s) : specs(s), relPos() {}
    virtual ~RangeSensor() {}
    
    virtual float getValue() = 0;
    math::vec2 getCoordinate() {
      return relPos.getPoint(getValue());
    }
  };
  
  class SensorFactory {
    protected:
    std::map<std::string, robot::RangeSpecs> rangeSensors;
    public:
    virtual ~SensorFactory() {}
    
    class WrongSensorKind {};
    
    virtual RangeSensor * rangeSensor(const std::string& name) = 0;
  };
  
  class MotorControl {
    public:
    virtual ~MotorControl() {}
    // speeds in cm / s
    virtual void setVelocity(float velocity) = 0;
    // radians / s
    virtual void setAngularVelocity(float angVel) = 0;
  };
  
  class MotorFactory {
    public:
    virtual ~MotorFactory() {}
    virtual MotorControl * newMotors() = 0;
  };
  
  class Robot {
    public:
    float size;
    std::map<std::string, RangeSensor*> rangeFinders;
    MotorControl * motors;
    std::vector<math::vec2> edges;
    std::vector<math::vec2> path;
    
    static const std::string NORTH;
    static const std::string SOUTH;
    static const std::string EAST;
    static const std::string WEST;
    
    math::Ray position;
    Robot();
    
    void act(); // do one iteration of its thang.
  }; // class SensorLayout
  
  Robot * read_robot(const char * path, SensorFactory * sensors, MotorFactory * motors);  
} // namespace robot

#endif // __ROBOT_H__