/*
 *  timers.h
 *  sim
 */

#include <sys/time.h>

#ifndef __TIMERS_H__
#define __TIMERS_H__

namespace robot {
  struct timeval time(void);
  // Returns the difference between two times in microseconds
  unsigned long time_diff(struct timeval first, struct timeval second);

} // namespace robot

#ifndef REAL_ROBOT
namespace sim {
  class Simulation;
  extern Simulation * curSim;
} // namespace sim
#endif // REAL_ROBOT

#endif // __TIMERS_H__