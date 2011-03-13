#include <stdint.h>
#include "robot.h"
#include "simulation.h"
#include "arduino.h"
#include "simsensors.h"
#include "simmotors.h"
#include "setup.h"
using namespace robot;

template<class mType1>
class HybridMotorControl : public MotorControl {
public:
  mType1 motors_0;
  sim::MotorControl motors_1;
  
  HybridMotorControl(sim::Simulation * w) : motors_0(), motors_1(w) {}
  
  virtual void setVelocity(float velocity) {
    motors_0.setVelocity(velocity);
    motors_1.setVelocity(velocity);
  }
  
  virtual void setAngularVelocity(float angVel) {
    motors_0.setAngularVelocity(angVel);
    motors_1.setAngularVelocity(angVel);
  }
  
  virtual void addMotor(Motor m) {
    motors_0.addMotor(m);
    motors_1.addMotor(m);
  }
};

class HybridSensorFactory : public SensorFactory {
public:
  SensorFactory * rangeFactory;
  SensorFactory * encoderFactory;
  
  HybridSensorFactory() throw()
    : rangeFactory(NULL), encoderFactory(NULL) { }
  
  virtual ~HybridSensorFactory() {
    if( rangeFactory ) {
      delete rangeFactory;
    }
    if( encoderFactory ) {
      delete encoderFactory;
    }
  }
  
  virtual RangeSensor * rangeSensor(const std::string& name) throw(WrongSensorKind) {
    return rangeFactory->rangeSensor(name);
  }
  
  virtual Encoder * encoder(const std::string& name) throw(WrongSensorKind) {
    return encoderFactory->encoder(name);
  }
};

sim::World * create_world(AbstractRobot * bot,
                  const char *mapPath, const char *botPath, const char *sensLibPath,
                  setupflags_t devices)
{
  HybridSensorFactory sensors;
  MotorControl * motors;
  sim::World * result = NULL;

  sim::Map * map = sim::read_map(mapPath);
  robot::MCL * mcl = new robot::MCL();
  mcl->initialize(map->start, 10, *map);
  bot->addSlam(mcl);
  
  // create the factories
  if( devices & SIM_REQUIRED ) {
    sim::Simulation * theSim = NULL;
    // If we have real motors and are actually moving,
    // then the simulation should be done in real time with real movement
    // If the motors are simulated,
    // then we're probably pushing the robot around
    // and we want discrete time intervals
    if( devices & REAL_MOTORS ) {
      theSim = new sim::RealTimeSimulation();
      if( devices & MOTORS_ARDUINO ) {
        motors = new HybridMotorControl<ArduinoMotors>(theSim);
      }
      else if( devices & MOTORS_MAESTRO ) {
        // motors = new HybridMotorControl<MaestroMotors, sim::MotorControl>();
      }
    }
    else if( devices & MOTORS_SIM ) {
      theSim = new sim::Simulation();
      motors = new sim::MotorControl(theSim);
    }
    
    result = theSim;
    theSim->map = map;
    theSim->bot = bot;
    theSim->simBot.position = theSim->map->start;
    theSim->simBot.lastPosition = theSim->simBot.position;

    if( devices & SONAR_SIM ) {
      sensors.rangeFactory = new sim::SensorFactory(sensLibPath, theSim);
    }
    
    if( devices & ENCODER_SIM ) {
      sensors.encoderFactory = new sim::SensorFactory(sensLibPath, theSim);
    }
  }
  
  if( devices & REAL_WORLD ) {
    if( ! (devices & SIM_REQUIRED) ) {
      result = new sim::RealWorld(bot);
    }
    if( devices & ARDUINO_REQUIRED ) {
      Arduino * arduino = new Arduino();
      
      if( devices & MOTORS_ARDUINO ) {
        if( ! (devices & SIM_REQUIRED) ) {
          motors = new ArduinoMotors(arduino);
        }
      }
      
      if( devices & SONAR_ARDUINO ) {
        //sensors.rangeFactory = new ArduinoSensorFactory();
      }
      
      if( devices & ENCODER_ARDUINO ) {
        //sensors.encoderFactory = new ArduinoSensorFactory();
      }
    }
    
    if( devices & MAESTRO_REQUIRED ) {
      // Maestro * maestro = new Maestro();
      
      // if( devices & MOTORS_MAESTRO ) {
      //   if( !( devices & SIM_REQUIRED) ) {
      //     motors = new MaestroMotorFactory();
      //   }
      // }
    }
  
  }
  
  bot->setMotorController(motors);
  read_robot(bot, botPath, &sensors);
  
  return result;
}
