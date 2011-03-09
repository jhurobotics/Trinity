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
    
    void setup(const char * path);
    void getSensor(sensorid_t id, char * value) throw(Serial::ReadError);
    void setMotor(motorid_t id, int32_t value) throw(Serial::WriteError);
    void setSensor(sensorid_t id, int32_t value) throw(Serial::WriteError);
    void switchLight(bool on);
  };
  
  class ArduinoMotors : public MotorControl {
  protected:
    // This class does NOT own the arduino class.
    // It expects it to be alive for the life of the Motors class
    // It also expects someone else to delete it when finished,
    // as other object are also likely sharing the same arduino object
    Arduino * arduino;
    float velocity;
    float angularVelocity;
    
    motorid_t left;
    motorid_t right;
    float distance;
    
    void sendCommand();
  public:
    ArduinoMotors(Arduino * _a = NULL) :
      arduino(_a), velocity(0), angularVelocity(0) {}
    virtual ~ArduinoMotors() {};
    
    void setVelocity(float v)  {
      velocity = v;
      sendCommand();
    }
    
    void setAngularVelocity(float aV) {
      angularVelocity = aV;
      sendCommand();
    }

    void setArduino(Arduino * a) {
      arduino = a;
    }
    
    void setLeftMotor(motorid_t l) {
      left = l | Arduino::MOTOR_FLAG;
    }

    void setRightMotor(motorid_t r) {
      right = r | Arduino::MOTOR_FLAG;
    }
    
    void setMotorDistance(float d) {
      distance = d;
      sendCommand();
    }
  };
  
  class ArduinoMotorFactory : public MotorFactory {
  protected:
    // Creates MotorControls that talk with this Arduino
    Arduino * arduino;
  public:
    ArduinoMotorFactory(Arduino * a = NULL) : arduino(a) {}
    
    void setArduino(Arduino * a) {
      arduino = a;
    }
    
    virtual MotorControl * newMotors() {
      return new ArduinoMotors(arduino);
    }
  };
}

#endif
