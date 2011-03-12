/*
 *  timers.c
 *  sim
 */
#include <sys/time.h>
#include "simulation.h"
#include "timers.h"
using namespace sim;

Simulation * sim::curSim = NULL;

unsigned long robot::micro_time(void) {
  if( curSim && !curSim->realTime() ) {
    return sim::curSim->time * 1000000;
  }
  else {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec*1000000 + tv.tv_usec;
  }
}

unsigned long robot::milli_time(void) {
  if( curSim && !curSim->realTime() ) {
    return sim::curSim->time * 1000;
  }
  else {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec*1000 + tv.tv_usec/1000.0;
  }
}
