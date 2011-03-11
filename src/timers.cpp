/*
 *  timers.c
 *  sim
 */
#include <ctime>
#include "simulation.h"
#include "timers.h"
using namespace sim;

Simulation * sim::curSim = NULL;

unsigned long robot::micro_time(void) {
  if( curSim && dynamic_cast<RealTimeSimulation*>(curSim) == NULL ) {
    return sim::curSim->time * 1000000;
  }
  else {
    return clock() * 1000000 / CLOCKS_PER_SEC;
  }
}

unsigned long robot::milli_time(void) {
  if( curSim && dynamic_cast<RealTimeSimulation*>(curSim) == NULL ) {
    return sim::curSim->time * 1000;
  }
  else {
    return clock() * 1000 / CLOCKS_PER_SEC;
  }
}

double robot::time(void) {
  if( curSim && dynamic_cast<RealTimeSimulation*>(curSim) == NULL ) {
    return sim::curSim->time;
  }
  else {
    return ((double)clock()) / ((double)CLOCKS_PER_SEC);
  }
}
