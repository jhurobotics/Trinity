/* -*-C++-*-
 * speedTest.cpp
 */
#include "arduino.h"
#include "speedTest.h"
#include "timers.h"
using namespace robot;
#include <iostream>
using namespace std;

void SpeedTest::act() {
  cout << "waiting for button\n";
  arduino->buttonWait();
  cout << "going!\n";
  control->setVelocity(10);
  struct timeval start = time();
  while( time_diff(start, time()) < 0.25 ) ;
  arduino->buttonWait();
  cout << "stopping!\n";
  halt();
}


void SpeedTest::halt() {
  control->setVelocity(0);
}
