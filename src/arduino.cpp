#include <limits.h>
#include <string.h>
#include "arduino.h"
#include "controllers.h"

#include <iostream>
using namespace robot;

void Arduino::setup(const char * path) throw(Serial::OpenError) {
  serial.Open(path);
}

void Arduino::getSensor_internal(sensorid_t id, char * value) throw(Serial::ReadError) {
  struct cmd {
    uint8_t name;
    uint8_t id;
    uint8_t end;
  };
  struct resp {
    uint8_t name;
    uint8_t id;
    char val[4];
    uint8_t end;
  };
  union {
    char buff[7];
    struct cmd cmd;
    struct resp resp;
  };
  cmd.name = GET;
  cmd.id = id;
  cmd.end = END;
  serial.Write(buff, 3);
  serial.Read(buff, 7);
  if( resp.name != GET || resp.id != id || resp.end != END ) {
    throw Serial::ReadError(-1);
  }
  memcpy(value, resp.val, 4);
}

void Arduino::setMotor(motorid_t id, int32_t value) throw(Serial::WriteError) {
  struct cmd {
    uint8_t name;
    uint8_t id;
    int32_t val;
    uint8_t end;
  };
  struct resp {
    uint8_t name;
    uint8_t id;
    uint8_t end;
  };
  union {
    char buff[7];
    struct cmd cmd;
    struct resp resp;
  };
  cmd.name = SET_MOTOR;
  cmd.id = id;
  cmd.val = value;  // Assuming little-endian
  cmd.end = END;
  serial.Write(buff, 7);
  serial.Read(buff, 3);
  if( resp.name != SET_MOTOR || resp.id != id || resp.end != END ) {
    throw Serial::WriteError(-1);
  }
}

void Arduino::setSensor(sensorid_t id, int32_t *value) throw(Serial::WriteError) {
  struct cmd {
    uint8_t name;
    uint8_t id;
    int32_t val;
    uint8_t end;
  };
  struct resp {
    uint8_t name;
    uint8_t id;
    int32_t val;
    uint8_t end;
  };
  union {
    char buff[7];
    struct cmd cmd;
    struct resp resp;
  };
  cmd.name = SET_SENSOR;
  cmd.id = id;
  cmd.val = *value;
  cmd.end = END;
  serial.Write(buff, 7);
  serial.Read(buff, 7);
  if( resp.name != SET_SENSOR || resp.id != id || resp.end != END ) {
    throw Serial::WriteError(-1);
  }
  (*value) = resp.val;
}

void Arduino::switchLight(bool on) throw(Serial::WriteError) {
  struct cmd {
    uint8_t name;
    uint8_t state;
    uint8_t end;
  };
  union {
    char buff[3];
    struct cmd cmd;
  };
  cmd.name = SET_LIGHT;
  cmd.state = (on ? 1 : 0 );
  cmd.end = END;
  serial.Write(buff, 3);
  serial.Read(buff, 3);
  if( cmd.name != SET_LIGHT || cmd.state != (on ? 1 : 0 ) || cmd.end != END ) {
    throw Serial::WriteError(-1);
  }
}

void Arduino::setFan(bool on) throw(Serial::WriteError) {
  struct cmd {
    uint8_t name;
    uint8_t state;
    uint8_t end;
  };
  union {
    char buff[3];
    struct cmd cmd;
  };
  cmd.name = SET_FAN;
  cmd.state = (on ? 1 : 0 );
  cmd.end = END;
  serial.Write(buff, 3);
  serial.Read(buff, 3);
  if( cmd.name != SET_FAN || cmd.state != (on ? 1 : 0 ) || cmd.end != END ) {
    throw Serial::WriteError(-1);
  }
}

void Arduino::buttonWait() throw(Serial::WriteError) {
  struct cmd {
    uint8_t name;
    uint8_t end;
  };
  union {
    char buff[2];
    struct cmd cmd;
  };
  cmd.name = BUTTON_WAIT;
  cmd.end = END;
  serial.Write(buff, 2);
  serial.Read(buff, 2);
  if( cmd.name != BUTTON_WAIT || cmd.end != END ) {
    throw Serial::WriteError(-1);
  }
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

float ArduinoSonar::getValue() {
  float distance;
  arduino->getSensor(id, &distance);
  return distance;
}

long ArduinoEncoder::getCount() {
  int32_t count;
  arduino->getSensor(id, &count);
  return count;
}

int ArduinoUV::getValue() {
  int32_t val;
  arduino->getSensor(id, &val);
  return val;
}
