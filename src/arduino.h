/* -*-C++-*-
 * Interface to the Arduino
 */
#include <stdint.h>
#include <assert.h>
#include "serial.h"
#include "robot.h"

#ifndef __ARDUINO_H__
#define __ARDUINO_H__

namespace robot {
  
  class Arduino {
  protected:
    Serial serial;
    void getSensor_internal(sensorid_t id, char * value) throw(Serial::ReadError);
  public:
    Arduino() throw() {}
    
    enum command_t{
      GET = 0x01,
      SET_MOTOR = 0x02,
      SET_SENSOR = 0x03,
      SET_LIGHT = 0x04,
      BUTTON_WAIT = 0x05,
      SET_FAN = 0x06,
    };
    
    static const motorid_t MOTOR_FLAG = 0x80;
    static const uint8_t END = 0xFF;
    
    void setup(const char * path) throw(Serial::OpenError);
    void setMotor(motorid_t id, int32_t value) throw(Serial::WriteError);
    void getSensor(sensorid_t id, float * value) throw(Serial::ReadError) {
      getSensor_internal(id, reinterpret_cast<char*>(value));
    }
    void getSensor(sensorid_t id, int32_t * value) throw(Serial::ReadError) {
      getSensor_internal(id, reinterpret_cast<char*>(value));
    }
    // Pass in the desired value
    // The value getting replaced is passed back out
    void setSensor(sensorid_t id, int32_t *value) throw(Serial::WriteError);
    void switchLight(bool on) throw(Serial::WriteError);
    void setFan(bool on) throw(Serial::WriteError);
    void buttonWait() throw(Serial::WriteError);
  };
  
  class ArduinoDevice {
  protected:
    Arduino * arduino;
  public:
    ArduinoDevice(Arduino * a = NULL) : arduino(a) { }
    void setArduino(Arduino * a) {
      arduino = a;
    }
  };
  
  class ArduinoSonar : public RangeSensor, public ArduinoDevice {
  public:
    explicit ArduinoSonar(const RangeSpecs& s, Arduino * a) 
      : RangeSensor(s), ArduinoDevice(a) { }
    virtual float getValue();    
  };
  
  class ArduinoEncoder : public Encoder, public ArduinoDevice {
  public:
    explicit ArduinoEncoder(float dist, Arduino * a = NULL)
      : Encoder(dist), ArduinoDevice(a) { }
    virtual long getCount();
  };
  
  class ArduinoUV : public UVSensor, public ArduinoDevice {
  public:
    explicit ArduinoUV(Arduino * a = NULL) : UVSensor(), ArduinoDevice(a) { }
    virtual int getValue();
  };
  
  class ArduinoSensorFactory : public SensorFactory, public ArduinoDevice {
  public:
    ArduinoSensorFactory(std::string libPath, Arduino * a = NULL)
      : SensorFactory(libPath), ArduinoDevice(a) { }
    
    virtual robot::RangeSensor * allocRange(const robot::RangeSpecs &specs) {
      return new ArduinoSonar(specs, arduino);
    }
    
    virtual robot::Encoder * allocEncoder(float td) {
      return new ArduinoEncoder(td, arduino);
    }
    
    virtual robot::UVSensor * uvsensor() throw(WrongSensorKind) {
      return new ArduinoUV(arduino);
    }
  };
}

#endif
