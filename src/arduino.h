/* -*-C++-*-
 * Interface to the Arduino
 */
#include <stdint.h>
#include "serial.h"
#include "robot.h"

#ifndef __ARDUNIO_H__
#define __ARDUINO_H__

namespace robot {
  
  typedef uint8_t id_t;
  typedef id_t motorid_t;
  typedef id_t sensorid_t;
  
  class Arduino {
  protected:
    Serial serial;
  public:
    Arduino() throw() {}
    
    enum command_t{
      GET = 0x01,
      SET_MOTOR = 0x02,
      SET_SENSOR = 0x03,
      LIGHT_SWITCH = 0x04,
    };
    
    static const motorid_t MOTOR_FLAG = 0x80;
    static const uint8_t END = 0xFF;
    
    void setup();
    void getSensor(sensorid_t id, char * value) throw(Serial::ReadError);
    void setMotor(motorid_t id, int32_t value) throw(Serial::WriteError);
    void setSensor(sensorid_t id, int32_t value) throw(Serial::WriteError);
    void switchLight(bool on);
  };
}

#endif
