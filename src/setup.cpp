#include <stdint.h>
#include <assert.h>
#include "robot.h"
#include "simulation.h"
#include "arduino.h"
#include "maestro.h"
#include "controllers.h"
#include "simsensors.h"
#include "simmotors.h"
#include "setup.h"
using namespace robot;

template<class mType1, class comm>
class HybridMotorControl : public MotorControl {
public:
  mType1 motors_0;
  sim::MotorControl motors_1;
  
  HybridMotorControl(sim::Simulation * w) : motors_0(), motors_1(w) {}
  HybridMotorControl(sim::Simulation * w, comm * c) : motors_0(c), motors_1(w) {}
  
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
  SensorFactory * uvFactory;
  
  virtual RangeSensor * allocRange(const RangeSpecs& specs) {
    return rangeFactory->allocRange(specs);
  }
  
  virtual Encoder * allocEncoder(float td) {
    return encoderFactory->allocEncoder(td);
  }
  
  HybridSensorFactory(std::string libPath) throw()
    : SensorFactory(libPath), rangeFactory(NULL), encoderFactory(NULL),
      uvFactory(NULL)
  { }
  
  virtual ~HybridSensorFactory() {
    if( rangeFactory ) {
      delete rangeFactory;
    }
    if( encoderFactory ) {
      delete encoderFactory;
    }
    if( uvFactory ) {
      delete uvFactory;
    }
  }
  
  virtual UVSensor * uvsensor() throw(WrongSensorKind) {
    return uvFactory->uvsensor();
  }
};

sim::World * create_world(AbstractRobot * bot,
                          const char *mapPath, const char *botPath, const char *sensLibPath,
                          setupflags_t devices, const char * arduinoPath,
                          const char * maestroPath)
{
  HybridSensorFactory sensors(sensLibPath);
  MotorControl * motors;
  sim::World * result = NULL;
  sim::Map * map = NULL;
  Arduino * arduino = NULL;
  if( devices & ARDUINO_REQUIRED ) {
    arduino = new Arduino();
    assert(arduinoPath != NULL);
    arduino->setup(arduinoPath);
  }
  Maestro * maestro = NULL;
  if( devices & MAESTRO_REQUIRED ) {
    maestro = new Maestro();
    assert(maestroPath != NULL);
    maestro->setup(maestroPath);
  }
  
  // create the factories
  if( devices & SIM_REQUIRED ) {
    map = sim::read_map(mapPath);
    sim::Simulation * theSim = NULL;
    // If we have real motors and are actually moving,
    // then the simulation should be done in real time with real movement
    // If the motors are simulated,
    // then we're probably pushing the robot around
    // and we want discrete time intervals
    if( devices & REAL_MOTORS ) {
      theSim = new sim::RealTimeSimulation();
      if( devices & MOTORS_ARDUINO ) {
        motors = new HybridMotorControl<ArduinoMotors, Arduino>(theSim, arduino);
      }
      else if( devices & MOTORS_MAESTRO ) {
        motors = new HybridMotorControl<MaestroMotors, Maestro>(theSim, maestro);
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
    
    if( devices & UV_SIM ) {
      sensors.uvFactory = new sim::SensorFactory(sensLibPath, theSim);
    }
  }
  
  if( devices & REAL_WORLD ) {
    if( ! (devices & SIM_REQUIRED) ) {
      result = new sim::RealWorld(bot);
    }
    if( devices & ARDUINO_REQUIRED ) {
      bot->setArduino(arduino);
      
      if( devices & MOTORS_ARDUINO ) {
        if( ! (devices & SIM_REQUIRED) ) {
          motors = new ArduinoMotors(arduino);
        }
      }
      
      if( devices & SONAR_ARDUINO ) {
        sensors.rangeFactory = new ArduinoSensorFactory(sensLibPath, arduino);
      }
      
      if( devices & ENCODER_ARDUINO ) {
        sensors.encoderFactory = new ArduinoSensorFactory(sensLibPath, arduino);
      }
      
      if( devices & UV_ARDUINO ) {
        sensors.uvFactory = new ArduinoSensorFactory(sensLibPath, arduino);
      }
    }
    
    if( devices & MAESTRO_REQUIRED ) {
      bot->setMaestro(maestro);
      
      if( devices & MOTORS_MAESTRO ) {
        if( !( devices & SIM_REQUIRED) ) {
          motors = new MaestroMotors();
        }
      }
    }
  }
  
  bot->setMotorController(motors);
  read_robot(bot, botPath, &sensors);
  
  return result;
}
