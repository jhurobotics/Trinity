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
#include "graph.h"
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
  
  class UVSensor : public Sensor {
  public:
    math::Ray relPos;
    explicit UVSensor() : relPos() { }
    virtual int getValue() = 0;
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
    virtual UVSensor * uvsensor() throw(WrongSensorKind) = 0;
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
    virtual void addUV(UVSensor * uv) = 0;
    virtual void setMotorController(MotorControl * control) = 0;
    virtual void addMotor(Motor motors) = 0;
    // Takes ownership of the graph, will delete it if replaced
    virtual void addGraph(Graph * g) {
      if( graph ) {
        delete graph;
      }
      graph = g;
    }
    virtual void addMap(sim::Map * m, std::string name) = 0;
    // Takes ownership of SLAM, will delete it if replaced
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
      HALLWAY_CHECK,
      SCAN,
			EXTINGUISH,
			GO_HOME,
    } curMode;
    
    enum HallChoices {
      ROOM1A,
      ROOM1B,
      ROOM4A,
      ROOM4B,
    } curDecision;

    math::vec2 doorDir;
    RangeSensor * decidingEye;
    float lastMeasurement;
    float totalMeasure;
    unsigned char decisionCount;
    
    enum ScanModes {
      START = 0,
      TURN_RIGHT,
      SCAN_LEFT,
      FINISHED,
      VERIFY_READING
    } curScanMode;
    
    ScanModes lastScanMode;
    timeval lastTime;
    int uvTotal;

		struct timeval extinguishTimer;

    Node * currentObjective;
    
#define map_1A 0x00
#define map_1B 0x01
#define map_4A 0x00
#define map_4B 0x10
    enum mapNames {
      map_1A_4A = map_1A | map_4A,
      map_1A_4B = map_1A | map_4B,
      map_1B_4A = map_1B | map_4A,
      map_1B_4B = map_1B | map_4B,
    };


    std::set<RangeSensor*> rangeFinders;
    typedef std::set<RangeSensor*>::iterator rangeIter_t;
    std::set<Encoder*> encoders;
    UVSensor * uvtron;
    MotorControl * control;

    float size;
    std::vector<math::vec2> edges;
    std::vector<math::vec2> path;


    //Store all the maps in a vector, so that we can switch between maps whenever necessary
    std::vector<sim::Map*> mapVector;
    int currentMapIndex;

  public:
    std::vector<math::vec2> realPoints;
  protected:
    int32_t uvBaselineCount;
    int32_t uvBaseline;
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
    void hallwayCheck() throw();
    void scan() throw();
    void extinguish() throw();
    void goHome() throw();
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
    virtual void addUV(UVSensor * uv) {
      if( uvtron ) {
        delete uvtron;
      }
      uvtron = uv;
    }
    
    virtual void setMotorController(MotorControl * c) {
      control = c;
    }
    
    virtual void addMotor(Motor m) {
      control->addMotor(m);
    }
    
    virtual void addMap(sim::Map* m, std::string name) {
      //ok. So lets name all the maps "1A_2A" and so on so forth. I will write code here based on this system.

      int i;
      //apparently we can use string comparisons using "==".
      if(name == "1A_4A")
        i = map_1A_4A;
      else if(name == "1A_4B")
        i = map_1A_4B;
      else if(name == "1B_4A")
        i = map_1B_4A;
      else if(name == "1B_4B")
        i = map_1B_4B;

      mapVector[i]= m;
      if( !slammer->isInitialized() ) {
        slammer->initialize(m->start, 10, m);
      }
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
    SPEEDTEST,
    CPP_1,
  };
  class BadRobotImplementation : public std::exception {};
  AbstractRobot * new_robot(Implementation imp) throw(BadRobotImplementation);
  void read_robot(AbstractRobot * bot, const char * path, SensorFactory * sensors);
} // namespace robot

#endif // __ROBOT_H__
