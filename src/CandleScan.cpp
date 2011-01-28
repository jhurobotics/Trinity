/*
 *  CandleScan.cpp
 *  sim
 */

#include "CandleScan.h"
using namespace robot;

// turn 180 degrees to the left in 8 seconds
#define SCAN_SPEED M_PI / 8

void CandleScanner::act() throw() {
  static float startAngle;
  static ScanModes curMode = START;
  float diff;
  switch( curMode ) {
    case START:
      startAngle = slammer->getPose().angle();
      motors->setVelocity(0);
      motors->setAngularVelocity( -SCAN_SPEED);
      curMode = TURN_RIGHT;
    case TURN_RIGHT:
      diff = startAngle - slammer->getPose().angle();
      if( diff < 0 ) {
        diff += 2*M_PI;
      }
      if( diff < M_PI / 2 ) {
        break;
      }
      else {
        // start the left scan
        curMode = SCAN_LEFT;
        motors->setAngularVelocity( SCAN_SPEED );
      }
    case SCAN_LEFT:
      // check to see if we saw anthing
      // if( see something ) {
      //  say something;
      //  curMode = VERIFY_READING;
      //  motors->setAngularVelocity(0);
      //  break;
      // }
      diff = slammer->getPose().angle() - startAngle;
      if( diff < 0 ) {
        diff += 2*M_PI;
      }
      if( diff < M_PI / 2 || diff > 3 * M_PI / 2) {
        break;
      }
      else {
        curMode = FINISHED;
        motors->setAngularVelocity(0);
      }
    case FINISHED:
      // return to hallway mode
      break;
    case VERIFY_READING:
      // No-op for now
      motors->setAngularVelocity(SCAN_SPEED);
      curMode = SCAN_LEFT;
      break;
  }
}
