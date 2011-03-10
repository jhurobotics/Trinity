#include <stdint.h>
#include "robot.h"
#include "simulation.h"
#include "arduino.h"
#include "simsensors.h"
#include "simmotors.h"
using namespace robot;

#define SIM_REQUIRED     0x1000
#define ARDUINO_REQUIRED 0x0100
#define MAESTRO_REQUIRED 0x0200

#define REAL_WORLD       0x0F00
#define COMM_MASK        0xFF00

#define MOTOR_MASK       0x00F0
#define SENSOR_MASK      0x000F

#define MOTORS_SIM       (SIM_REQUIRED     | 0x0080)
#define MOTORS_ARDUINO   (ARDUINO_REQUIRED | 0x0010)
#define MOTORS_MAESTRO   (MAESTRO_REQUIRED | 0x0020)
#define REAL_MOTORS      (MOTORS_ARDUINO | MOTORS_MAESTRO)

#define SONAR_SIM        (SIM_REQUIRED     | 0x0001)
#define SONAR_ARDUINO    (ARDUINO_REQUIRED | 0x0002)
#define ENCODER_SIM      (SIM_REQUIRED     | 0x0004)
#define ENCODER_ARDUINO  (ARDUINO_REQUIRED | 0x0008)

typedef uint16_t setupflags_t;

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
};

template<class mType1>
class HybridMotorFactory : public robot::MotorFactory {
protected:
  sim::Simulation * world;
public:
  HybridMotorFactory(sim::Simulation * w) : world(w) {}
  virtual MotorControl * newMotors() {
    return new HybridMotorControl<mType1>(world);
  }
};

class HybridSensorFactory : public SensorFactory {
public:
  SensorFactory * rangeFactory;
  SensorFactory * encoderFactory;
  virtual RangeSensor * rangeSensor(const std::string& name) throw(WrongSensorKind) {
    return rangeFactory->rangeSensor(name);
  }
  virtual Encoder * encoder(const std::string& name) throw(WrongSensorKind) {
    return encoderFactory->encoder(name);
  }
};

sim::Simulation * create_world(AbstractRobot * bot,
                  const char *mapPath, const char *botPath, const char *sensLibPath,
                  setupflags_t devices)
{
  HybridSensorFactory sensors;
  MotorFactory * motors;
  sim::Simulation * result = NULL;

  sim::Map * map = sim::read_map(mapPath);
  robot::MCL * mcl = new robot::MCL();
  mcl->initialize(map->start, 10, *map);
  bot->addSlam(mcl);
  
  // create the factories
  if( devices & SIM_REQUIRED ) {
    result = new sim::Simulation();
    result->map = map;
    result->bot.bot = bot;
    result->bot.position = result->map->start;
    result->bot.lastPosition = result->bot.position;
    
    if( devices & REAL_MOTORS ) {
      if( devices & MOTORS_ARDUINO ) {
        motors = new HybridMotorFactory<ArduinoMotors>(result);
      }
      else if( devices & MOTORS_MAESTRO ) {
        // motors = new HybridMotorControl<MaestroMotors, sim::MotorControl>();
      }
    }
    
    if( devices & SONAR_SIM ) {
      sensors.rangeFactory = new sim::SensorFactory(sensLibPath, result);
    }
    
    if( devices & ENCODER_SIM ) {
      sensors.encoderFactory = new sim::SensorFactory(sensLibPath, result);
    }
  }
  
  if( devices & REAL_WORLD ) {
    if( devices & ARDUINO_REQUIRED ) {
      Arduino * arduino = new Arduino();
      
      if( devices & MOTORS_ARDUINO ) {
        if( ! (devices & SIM_REQUIRED) ) {
          motors = new ArduinoMotorFactory(arduino);
        }
      }
      
      if( devices & SONAR_ARDUINO ) {
        //sensors.rangeFactory = new ArduinoSensorFactory();
      }
      
      if( devices & ENCODER_ARDUINO ) {
        //sensros.encoderFactory = new ArduinoSensorFactory();
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
  
  
  read_robot(bot, botPath, &sensors, motors);
  
  delete motors;
  return result;
}
