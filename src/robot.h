/* -*-C++-*-
 *  robot.h
 *  sim
 */

#include <vector>
#include <map>
#include <set>
#include <string>
#include <stdint.h>
#include "geometry.h"
#include "Nodel.h"
#include "slam/slam.h"
#include "timers.h"

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
  
  typedef uint8_t id_t;
  typedef id_t motorid_t;
  typedef id_t sensorid_t;
  
  class Sensor {
  public:
    uint8_t id;
  };
  
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
  
  class RangeSensor : public Sensor {
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
  
  class Encoder : public Sensor {
    public:
    math::vec2 relPos;
    float tickDist;
    explicit Encoder(float td) : tickDist(td) { }
    virtual long getCount() = 0;
  };
  
  class SensorFactory {
  protected:
    std::string libPath;
    std::map<std::string, robot::RangeSpecs> rangeSensors;
    std::map<std::string, float> encoders;
    
  public:
    virtual RangeSensor * allocRange(const robot::RangeSpecs& specs) = 0;
    virtual Encoder * allocEncoder(float tickDist) = 0;
    
    SensorFactory(std::string path) : libPath(path) { }
    virtual ~SensorFactory() {}
    
    class WrongSensorKind {};
    
    virtual RangeSensor * rangeSensor(const std::string& name) throw(WrongSensorKind);
    virtual Encoder * encoder(const std::string& name) throw(WrongSensorKind);
  };  // class SensorFactory
  
  struct Motor {
    uint8_t id;
    math::Ray relPos;
    float minSpeed;
    float maxSpeed;
  };
  
  class MotorControl {
    public:
    virtual ~MotorControl() {}
    // speeds in cm / s
    virtual void setVelocity(float velocity) = 0;
    // radians / s
    virtual void setAngularVelocity(float angVel) = 0;
    virtual void addMotor(Motor m) = 0;
  };  // class MotorControl
  
  class MotorFactory {
    public:
    virtual ~MotorFactory() {}
    virtual MotorControl * newMotors(const char * name) = 0;
  };  // class MotorFactory
  
  class AbstractRobot {
    protected:
    math::Graph * graph;
    SLAM * slammer;
    Arduino * arduino;
    Maestro * maestro;
    
    public:
    AbstractRobot()
      : graph(NULL), slammer(NULL), arduino(NULL), maestro(NULL) {}
    virtual ~AbstractRobot();
    
    math::Graph * getGraph() const {
      return graph;
    }
    SLAM * getSlam() const {
      return slammer;
    }
    
    virtual void act() = 0; // do one iteration of its thang.
    virtual void addRangeSensor(RangeSensor * sensor) = 0;
    virtual void addEncoder(Encoder * encoder) = 0;
    virtual void setMotorController(MotorControl * control) = 0;
    virtual void addMotor(Motor motors) = 0;
    // Takes ownership of the graph, will delete it if replaced
    virtual void addGraph(math::Graph * g) {
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
    void setArduino(Arduino * a);
    void setMaestro(Maestro * m);
  }; // class AbstractRobot
  
  class SonarRobot : public AbstractRobot {
  protected:
    enum BotModes {
      INIT,
      HALLWAY,
      SCAN
    } curMode;
    enum ScanModes {
      START = 0,
      TURN_RIGHT,
      SCAN_LEFT,
      FINISHED,
      VERIFY_READING
    } curScanMode;
    math::Node * currentObjective;
    
    std::set<RangeSensor*> rangeFinders;
    std::set<Encoder*> encoders;
    MotorControl * control;
    
    float size;
    std::vector<math::vec2> edges;
    std::vector<math::vec2> path;
  public:
    std::vector<math::vec2> realPoints;
  protected:
    math::Ray position;
    struct timeval lastSLAMTime;
    std::vector<long> encoderCounts;
  public:
    SonarRobot() throw();
    virtual ~SonarRobot() {
      if( control ) {
        delete control;
      }
      std::set<RangeSensor*>::iterator end = rangeFinders.end();
      for( std::set<RangeSensor*>::iterator iter = rangeFinders.begin(); iter != end; iter++ ) {
        delete *iter;
      }
    }
    
    virtual void act() throw(); // do one iteration of its thang.
  protected:
    void hallway() throw();
    void scan() throw();
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
    
    virtual void setMotorController(MotorControl * c) {
      control = c;
    }
    
    virtual void addMotor(Motor m) {
      control->addMotor(m);
    }

    virtual void draw();
    virtual const math::Ray& getPosition() throw() {
      return position;
    }
    virtual void halt() {
      if( control ) {
        control->setVelocity(0);
        control->setAngularVelocity(0);
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
  void read_robot(AbstractRobot * bot, const char * path, SensorFactory * sensors);
} // namespace robot

#endif // __ROBOT_H__
