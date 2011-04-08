/* -*-C++-*-
 * speedTest.h
 */

#include "robot.h"

#ifndef __SPEEDTEST_H__
#define __SPEEDTEST_H__

namespace robot {
  class SpeedTest : public AbstractRobot {
  protected:
    MotorControl * control;
  public:
    virtual void act();
    virtual void addRangeSensor(RangeSensor * sensor) {
      delete sensor;
    }
    virtual void addEncoder(Encoder * encoder) {
      delete encoder;
    }
    virtual void setMotorController(MotorControl * c) {
      control = c;
    }
    virtual void addMotor(Motor motors) {
      control->addMotor(motors);
    }
    virtual void addMap(sim::Map * m, std::string name) {
      delete m;
    }
    virtual const math::Ray& getPosition() {
      static math::Ray origin;
      return origin;
    }
    virtual void halt();
  };
}
#endif // __SPEEDTEST_H__
