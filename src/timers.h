/*
 *  timers.h
 *  sim
 */

#ifndef __TIMERS_H__
#define __TIMERS_H__

namespace robot {
  // returns the time in microseconds
  unsigned long micro_time(void);
  // returns the time in milliseconds
  unsigned long milli_time(void);
  // returns the time in seconds
  double time(void);
} // namespace robot

#ifndef REAL_ROBOT
namespace sim {
  class Simulation;
  extern Simulation * curSim;
} // namespace sim
#endif // REAL_ROBOT

#endif // __TIMERS_H__