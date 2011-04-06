//
//  maestro.cpp
//  sim
//

#include <limits.h>
#include "maestro.h"
#include "controllers.h"
using namespace robot;

void Maestro::setup(const char *path) {
  serial.Open(path);
}

void Maestro::setChannel(id_t channel, uint16_t value) {
  value *= 4;
  unsigned char command[] = {
    0x84, channel, value & 0x7F, (value >> 7) & 0x7F
  };
  serial.Write((const char*)command, 4);
}

static inline uint16_t convertVelocity(float vel, Motor m) {
  return static_cast<uint16_t>((vel - m.minSpeed)/(m.maxSpeed - m.minSpeed) + 1) * 4000;
}

void MaestroMotors::sendCommand() {
  if( !maestro ) {
    return;
  }
  float leftVel = velocity - distance * angularVelocity;
  float rightVel = velocity + distance * angularVelocity;
  maestro->setChannel(left.id, convertVelocity(leftVel, left));
  maestro->setChannel(right.id, convertVelocity(rightVel, right));
}
