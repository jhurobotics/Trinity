/*
 *  CandleScan.h
 *  sim
 */

#include "robot.h"

#ifndef __CANDLE_SCAN_H__
#define __CANDLE_SCAN_H__

namespace robot {
  
  class CandleScanner : public AbstractRobot {
    enum ScanModes {
      START = 0,
      TURN_RIGHT,
      SCAN_LEFT,
      FINISHED,
      VERIFY_READING
    };
    MotorControl * motors;
    
    public:
    CandleScanner() throw() : motors(NULL) {}
    virtual ~CandleScanner() {
      if( motors ) {
        delete motors;
      }
    }
    
    virtual void act() throw();
    public:
    virtual void addRangeSensor(RangeSensor * sensor) {
      delete sensor;
    }
    virtual void addMotors(MotorControl * m) {
      motors = m;
    }
  }; // class CandleScanner
  
} // namespace robot

#endif // __CANDLE_SCAN_H__