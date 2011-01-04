/*
 *  robot.h
 *  sim
 */

#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <vector>
#include <map>
#include "geometry.h"

namespace robot {
  
  enum SensorKind {
    UNKNOWN = 0,
    RANGE
  };
  
  class Sensor {
    public:
    Sensor() {}
    virtual ~Sensor() {}
  }; // class Sensor
  
  class SingleValueSensor : public Sensor {
    public:
    SingleValueSensor() : Sensor() {}
    virtual ~SingleValueSensor() {}
    
    virtual float getValue() = 0;
  }; // class SingleValueSensor
  
  class RangeSensor : public SingleValueSensor {
    public:
    // Position relative to the robot
    math::Ray relPos;
    RangeSensor() : SingleValueSensor(), relPos() {}
    virtual ~RangeSensor() {}
  };
  
  class SensorFactory {
    public:
    virtual ~SensorFactory() {}
    virtual Sensor * newSensorWithName(const char * name, robot::SensorKind* kind = NULL) = 0;
    
    class WrongSensorKind {
    };

    virtual RangeSensor * rangeSensor(const char * name) throw(WrongSensorKind) = 0;
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
    std::map<math::Ray, RangeSensor*> rangeFinders;
    MotorControl * motors;
    std::vector<math::vec2> edges;
    std::vector<math::vec2> path;
    
    const math::Ray NORTH;
    const math::Ray SOUTH;
    const math::Ray EAST;
    const math::Ray WEST;
    
    math::Ray position;
    Robot();
    
    void act(); // do one iteration of its thang.
  }; // class SensorLayout
  
  Robot * read_robot(const char * path, SensorFactory * sensors, MotorFactory * motors);  
} // namespace robot

#endif // __ROBOT_H__