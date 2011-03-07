#include <string.h>
#include "arduino.h"
using namespace robot;

void Arduino::getSensor(uint8_t id, char * value) throw(Serial::ReadError) {
  uint8_t buff[7];
  buff[0] = GET;
  buff[1] = id;
  buff[2] = END;
  serial.Write((char*)buff, 3);
  serial.Read((char*)buff, 7);
  if( buff[0] != GET || buff[1] != id || buff[7] != END ) {
    throw Serial::ReadError();
  }
  memcpy(value, buff+2, 4);
}

void Arduino::setMotor(uint8_t id, int32_t value) throw(Serial::WriteError) {
  uint8_t buff[7];
  buff[0] = SET_MOTOR;
  buff[1] = id;
  buff[2] = value & 0xFF;
  buff[3] = ( value >> 8 ) & 0xFF;
  buff[4] = ( value >> 16) & 0xFF;
  buff[5] = ( value >> 24) & 0xFF;
  buff[6] = END;
  serial.Write((char*)buff, 7);
  serial.Read((char*)buff, 3);
  if( buff[0] != SET_MOTOR || buff[1] != id || buff[2] != END ) {
    throw Serial::WriteError();
  }
}

void Arduino::setSensor(uint8_t id, int32_t value) throw(Serial::WriteError) {
  char buff[7];
  buff[0] = SET_SENSOR;
  buff[1] = id;
  buff[2] = value & 0xFF;
  buff[3] = ( value >> 8 ) & 0xFF;
  buff[4] = ( value >> 16) & 0xFF;
  buff[5] = ( value >> 24) & 0xFF;
  buff[6] = END;
  serial.Write(buff, 7);
  serial.Read(buff, 3);
  if( buff[0] != SET_SENSOR || buff[1] != id || buff[2] != END ) {
    throw Serial::WriteError();
  }
}

void Arduino::switchLight(bool on) {
}
