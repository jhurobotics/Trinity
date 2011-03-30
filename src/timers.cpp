/*
 *  timers.c
 *  sim
 */
#include "simulation.h"
#include "timers.h"

#define MILLION 1000000

sim::Simulation * sim::curSim = NULL;

struct timeval robot::time(void) {
  struct timeval tv;
  if( sim::curSim && !sim::curSim->realTime() ) {
    tv.tv_sec = (unsigned long)sim::curSim->time;
    tv.tv_usec = (unsigned long)(sim::curSim->time * MILLION);
  }
  else {
    gettimeofday(&tv, NULL);
  }
  return tv;
}

#ifndef timevalcmp
#define timevalcmp(tvp, uvp, cmp) \
 (((tvp)->tv_sec == (uvp)->tv_sec) ?				\
 ((tvp)->tv_usec cmp (uvp)->tv_usec) :			\
 ((tvp)->tv_sec cmp (uvp)->tv_sec))
#endif

// returns the difference between two times in microseconds
unsigned long robot::time_diff(struct timeval first, struct timeval second) {
  if( timevalcmp(&first, &second, >) ) {
    return robot::time_diff(second, first);
  }
  unsigned long diff = 0;
  diff = second.tv_sec - first.tv_sec;
  diff *= MILLION;
  diff += second.tv_usec - first.tv_usec;
  return diff;
}
