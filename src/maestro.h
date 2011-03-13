//
//  maestro.h
//  sim
//

#include "serial.h"
#include "robot.h"

#ifndef __MAESTRO_H__
#define __MAESTRO_H__

namespace robot {
  
  class Maestro {
  protected:
    Serial serial;
    
  public:
    Maestro() throw() { };
    
    void setup(const char * path);
    void setMotor(id_t channel, uint16_t value);
  };
  
  class MaestroMotors : public MotorControl {
  protected:
    Maestro * maestro;
    float velocity;
    float angularVelocity;
    
    Motor left;
    Motor right;
    float distance;
    
    void sendCommand();
  public:
    MaestroMotors(Maestro * m = NULL) :
      maestro(m), velocity(0), angularVelocity(0), distance(0) {}
    virtual ~MaestroMotors() { }
    
    void setVelocity(float v) {
      velocity = v;
      sendCommand();
    }
    
    void setAngularVelocity(float aV) {
      angularVelocity = aV;
      sendCommand();
    }
    
    virtual void addMotor(Motor m) {
      m.id -- ;
      if( m.relPos.origin().x < 0 ) {
        left = m;
      }
      else {
        right = m;
      }
    }
    
    void setMaestro(Maestro * m) {
      maestro = m;
    }
    
    void setMotorDistance(float d) {
      distance = d;
      sendCommand();
    }
  };
}

#endif // __MAESTRO_H__