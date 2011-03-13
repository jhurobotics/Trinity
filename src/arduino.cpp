#include <limits.h>
#include <string.h>
#include "arduino.h"
#include "controllers.h"
using namespace robot;

void Arduino::setup(const char * path) {
  serial.Open(path);
}

void Arduino::getSensor(sensorid_t id, char * value) throw(Serial::ReadError) {
  uint8_t buff[7];
  buff[0] = GET;
  buff[1] = id;
  buff[2] = END;
  serial.Write(reinterpret_cast<char*>(buff), 3);
  serial.Read(reinterpret_cast<char*>(buff), 7);
  if( buff[0] != GET || buff[1] != id || buff[7] != END ) {
    throw Serial::ReadError();
  }
  memcpy(value, buff+2, 4);
}

void Arduino::setMotor(motorid_t id, int32_t value) throw(Serial::WriteError) {
  uint8_t buff[7];
  buff[0] = SET_MOTOR;
  buff[1] = id;
  buff[2] = value & 0xFF;
  buff[3] = ( value >> 8 ) & 0xFF;
  buff[4] = ( value >> 16) & 0xFF;
  buff[5] = ( value >> 24) & 0xFF;
  buff[6] = END;
  serial.Write(reinterpret_cast<char*>(buff), 7);
  serial.Read(reinterpret_cast<char*>(buff), 3);
  if( buff[0] != SET_MOTOR || buff[1] != id || buff[2] != END ) {
    throw Serial::WriteError();
  }
}

void Arduino::setSensor(sensorid_t id, int32_t value) throw(Serial::WriteError) {
  uint8_t buff[7];
  buff[0] = SET_SENSOR;
  buff[1] = id;
  buff[2] = value & 0xFF;
  buff[3] = ( value >> 8 ) & 0xFF;
  buff[4] = ( value >> 16) & 0xFF;
  buff[5] = ( value >> 24) & 0xFF;
  buff[6] = END;
  serial.Write(reinterpret_cast<char*>(buff), 7);
  serial.Read(reinterpret_cast<char*>(buff), 3);
  if( buff[0] != SET_SENSOR || buff[1] != id || buff[2] != END ) {
    throw Serial::WriteError();
  }
}

void Arduino::switchLight(bool on) {
}

static inline int32_t convertVelocity(float vel, Motor m) {
  return static_cast<int32_t>((vel - m.minSpeed)/(m.maxSpeed - m.minSpeed)) * INT_MAX;
}

void ArduinoMotors::sendCommand() {
  if( !arduino ) {
    return;
  }
  float leftVel = velocity - distance * angularVelocity;
  float rightVel = velocity + distance * angularVelocity;
  arduino->setMotor(left.id, convertVelocity(leftVel, left));
  arduino->setMotor(right.id, convertVelocity(rightVel, right));
}

