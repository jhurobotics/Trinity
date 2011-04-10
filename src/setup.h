// -*- C++ -*-
//  setup.h
//  sim
//

#ifndef __SETUP_H__
#define __SETUP_H__

#define SIM_REQUIRED     0x10000
#define ARDUINO_REQUIRED 0x01000
#define MAESTRO_REQUIRED 0x02000

#define REAL_WORLD       0x0F000
#define COMM_MASK        0xFF000

#define MOTOR_MASK       0x00F00
#define SENSOR_MASK      0x000FF

#define MOTORS_SIM       (SIM_REQUIRED     | 0x00800)
#define MOTORS_ARDUINO   (ARDUINO_REQUIRED | 0x00100)
#define MOTORS_MAESTRO   (MAESTRO_REQUIRED | 0x00200)
#define REAL_MOTORS      (MOTORS_ARDUINO | MOTORS_MAESTRO)

#define SONAR_SIM        (SIM_REQUIRED     | 0x00001)
#define SONAR_ARDUINO    (ARDUINO_REQUIRED | 0x00002)
#define ENCODER_SIM      (SIM_REQUIRED     | 0x00004)
#define ENCODER_ARDUINO  (ARDUINO_REQUIRED | 0x00008)
#define UV_SIM           (SIM_REQUIRED     | 0x00010)
#define UV_ARDUINO       (ARDUINO_REQUIRED | 0x00020)

typedef uint32_t setupflags_t;

#ifndef __ROBOT_H__
namespace robot {
  class AbstractRobot;
}
#endif // __ROBOT_H__

#ifndef __SIMULATION_H__
namespace sim {
  class World;
}
#endif // __SIMULATION_H__

sim::World * create_world(robot::AbstractRobot * bot,
                          const char *mapPath, const char *botPath, const char *sensLibPath,
                          setupflags_t devices, const char * arduinoPath,
                          const char * maestroPath);

#endif
