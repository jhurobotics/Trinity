//
//  controllers.h
//  sim
//

#include "serial.h"
#include "robot.h"
#include "arduino.h"

#ifndef __CONTROLLERS_H__
#define __CONTROLLERS_H__

namespace robot {

#ifndef __MAESTRO_H__
  class Maestro;
#endif
  
  class DualWheelControl : public MotorControl {
  protected:
    float velocity;
    float angularVelocity;
    
    Motor left;
    Motor right;
    float distance;
    
    virtual void sendCommand() = 0;
    
  public:
    DualWheelControl() throw() :
      velocity(0), angularVelocity(0), left(), right(), distance(0) {}
    virtual ~DualWheelControl() { } 
        
    virtual void addMotor(Motor m) {
      if( m.relPos.origin().x < 0 ) {
        left = m;
      }
      else {
        right = m;
      }
      distance = (left.relPos.origin() - right.relPos.origin()).mag();
    }
    
    void setVelocity(float v) {
      velocity = v;
      sendCommand();
    }
    
    void setAngularVelocity(float aV) {
      angularVelocity = aV;
      sendCommand();
    }
  };
  
  class ArduinoMotors : public DualWheelControl {
  protected:
    Arduino * arduino;
    
    virtual void sendCommand();
    
  public:
    ArduinoMotors(Arduino * a = NULL) :
      DualWheelControl(), arduino(a) { }
    virtual ~ArduinoMotors() { }
    
    virtual void addMotor(Motor m) {
      m.id |= Arduino::MOTOR_FLAG;
      DualWheelControl::addMotor(m);
    }
    
    void setArduino(Arduino * a) {
      arduino = a;
    }
  };
  
  class MaestroMotors : public DualWheelControl {
  protected:
    Maestro * maestro;
    
    virtual void sendCommand();
    
  public:
    MaestroMotors(Maestro * m = NULL) :
      DualWheelControl(), maestro(m) { }
    virtual ~MaestroMotors() { }
    
    virtual void addMotor(Motor m) {
      m.id --;
      DualWheelControl::addMotor(m);
    }
    
    void setMaestro(Maestro * m) {
      maestro = m;
    }
  };
  
} // namespace robot

#endif // __CONTROLLERS_H__