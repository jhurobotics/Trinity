/* -*-C++-*-
 *  robot.h
 *  sim
 */

#include <vector>
#include <map>
#include <set>
#include <string>
#include "geometry.h"
#include "graph.h"
#include "slam/slam.h"

namespace robot {
#ifndef __ARDUINO_H__
  class Arduino;
#endif
#ifndef __MAESTRO_H__
  class Maestro;
#endif
} // namespace robot

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
  
  class Encoder {
    public:
    math::vec2 relPos;
    float tickDist;
    explicit Encoder(float td) : tickDist(td) { }
    virtual long getCount() = 0;
  };
  
  class SensorFactory {
    protected:
    std::map<std::string, robot::RangeSpecs> rangeSensors;
    std::map<std::string, float> encoders;
    public:
    virtual ~SensorFactory() {}
    
    class WrongSensorKind {};
    
    virtual RangeSensor * rangeSensor(const std::string& name) throw(WrongSensorKind) = 0;
    virtual Encoder * encoder(const std::string& name) throw(WrongSensorKind) = 0;
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
    Arduino * arduino;
    Maestro * maestro;
    
    public:
    AbstractRobot()
      : graph(NULL), slammer(NULL), arduino(NULL), maestro(NULL) {}
    virtual ~AbstractRobot();
    
    Graph * getGraph() const {
      return graph;
    }
    SLAM * getSlam() const {
      return slammer;
    }
    
    virtual void act() = 0; // do one iteration of its thang.
    virtual void addRangeSensor(RangeSensor * sensor) = 0;
    virtual void addEncoder(Encoder * encoder) = 0;
    virtual void addMotors(MotorControl * motors) = 0;
    // Takes ownership of the graph, will delete it if replaced
    virtual void addGraph(Graph * g) {
      if( graph ) {
        delete graph;
      }
      graph = g;
    }
    // Takes ownership of SLAM, will delete it if replaced
    virtual void addSlam(SLAM * s) {
      if( slammer ) {
        delete slammer;
      }
      slammer = s;
    }
    virtual void draw() { }
    virtual const math::Ray& getPosition() = 0;
    virtual void halt() = 0;
  }; // class AbstractRobot
  
  class SonarRobot : public AbstractRobot {
  protected:
    enum BotModes {
      HALLWAY,
      SCAN
    } curMode;
    Node * currentObjective;
    
    std::set<RangeSensor*> rangeFinders;
    std::set<Encoder*> encoders;
    MotorControl * motors;
    
    float size;
    std::vector<math::vec2> edges;
    std::vector<math::vec2> path;
  public:
    std::vector<math::vec2> realPoints;
  protected:
    math::Ray position;
    unsigned long lastSLAMTime;
    std::vector<long> encoderCounts;
  public:
    SonarRobot() throw();
    virtual ~SonarRobot() {
      if( motors ) {
        delete motors;
      }
      std::set<RangeSensor*>::iterator end = rangeFinders.end();
      for( std::set<RangeSensor*>::iterator iter = rangeFinders.begin(); iter != end; iter++ ) {
        delete *iter;
      }
    }
    
    virtual void act() throw(); // do one iteration of its thang.
  protected:
    void hallway() throw();
  public:
    // SLAM is added before reading the config
    virtual void addRangeSensor(RangeSensor * sensor) {
      rangeFinders.insert(sensor);
      slammer->addRangeSensor(sensor);
    }
    virtual void addEncoder(Encoder * encoder) {
      encoders.insert(encoder);
      slammer->addEncoder(encoder);
    }

    virtual void addMotors(MotorControl * m) {
      motors = m;
    }

    virtual void draw();
    virtual const math::Ray& getPosition() throw() {
      return position;
    }
    virtual void halt() {
      if( motors ) {
        motors->setVelocity(0);
        motors->setAngularVelocity(0);
      }
    }
  }; // class SonarRobot
  
  enum Implementation {
    SONAR,
    CPP_1,
    CPP_2
  };
  class BadRobotImplementation : public std::exception {};
  AbstractRobot * new_robot(Implementation imp) throw(BadRobotImplementation);
  void read_robot(AbstractRobot * bot, const char * path, SensorFactory * sensors, MotorFactory * motors);
} // namespace robot

#endif // __ROBOT_H__
