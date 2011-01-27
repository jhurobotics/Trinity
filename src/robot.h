/*
 *  robot.h
 *  sim
 */

#include <vector>
#include <map>
#include <set>
#include <string>
#include "geometry.h"
#include "graph.h"
#include "slam.h"

#ifndef __ROBOT_H__
#define __ROBOT_H__

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
  }; // struct RangeSpecs
  
  class RangeSensor {
    public:
    RangeSpecs specs;
    // Position relative to the robot
    math::Ray relPos;
    explicit RangeSensor(const RangeSpecs& s) : specs(s), relPos() {}
    explicit RangeSensor(const RangeSensor& s) : specs(s.specs), relPos(s.relPos) {}
    virtual ~RangeSensor() {}
    
    virtual float getValue() = 0;
    math::vec2 getCoordinate() {
      return relPos.getPoint(getValue());
    }
  }; // class RangeSensor
  
  class SensorFactory {
    protected:
    std::map<std::string, robot::RangeSpecs> rangeSensors;
    public:
    virtual ~SensorFactory() {}
    
    class WrongSensorKind {};
    
    virtual RangeSensor * rangeSensor(const std::string& name) = 0;
  };  // class SensorFactory
  
  class MotorControl {
    public:
    virtual ~MotorControl() {}
    // speeds in cm / s
    virtual void setVelocity(float velocity) = 0;
    // radians / s
    virtual void setAngularVelocity(float angVel) = 0;
  };  // class MotorControl
  
  class MotorFactory {
    public:
    virtual ~MotorFactory() {}
    virtual MotorControl * newMotors() = 0;
  };  // class MotorFactory
  
  class AbstractRobot {
    protected:
    Graph * graph;
    SLAM * slammer;
    
    public:
    AbstractRobot() {}
    virtual ~AbstractRobot() {
      if( graph ) {
        delete graph;
      }
      if( slammer ) {
        delete slammer;
      }
    }
    
    Graph * getGraph() const {
      return graph;
    }
    SLAM * getSlam() const {
      return slammer;
    }
    
    virtual void act() = 0; // do one iteration of its thang.
    virtual void addRangeSensor(RangeSensor * sensor) = 0;
    virtual void addMotors(MotorControl * motors) = 0;
    virtual void addGraph(Graph * g) {
      graph = g;
    }
    virtual void addSlam(SLAM * s) {
      slammer = s;
    }
    virtual void draw() { }
  }; // class AbstractRobot
  
  class SonarRobot : public AbstractRobot {
    std::set<RangeSensor*> rangeFinders;
    MotorControl * motors;
    public:
    float size;
    std::vector<math::vec2> edges;
    std::vector<math::vec2> path;
    std::vector<math::vec2> realPoints;
    
    math::Ray position;
    SonarRobot() throw();
    
    virtual void act() throw(); // do one iteration of its thang.
    virtual void addRangeSensor(RangeSensor * sensor) {
      rangeFinders.insert(sensor);
    }

    virtual void addMotors(MotorControl * m) {
      motors = m;
    }

    virtual void draw();
  }; // class SonarRobot
  
  enum Implementation {
    PYTHON,
    SONAR,
    CPP_1,
    CPP_2
  };
  class BadRobotImplementation : public std::exception {};
  AbstractRobot * new_robot(Implementation imp, const char * path = NULL) throw(BadRobotImplementation);
  void read_robot(AbstractRobot * bot, const char * path, SensorFactory * sensors, MotorFactory * motors);
} // namespace robot

#endif // __ROBOT_H__
