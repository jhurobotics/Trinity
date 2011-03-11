// -*- C++ -*-
//  setup.h
//  sim
//

#ifndef __SETUP_H__
#define __SETUP_H__

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

#ifndef __ROBOT_H__
namespace robot {
  class AbstractRobot;
}
#endif // __ROBOT_H__

#ifndef __SIMULATION_H__
namespace sim {
  class Simulation;
}
#endif // __SIMULATION_H__

sim::Simulation * create_world(robot::AbstractRobot * bot,
                               const char *mapPath, const char *botPath, const char *sensLibPath,
                               setupflags_t devices);

#endif
