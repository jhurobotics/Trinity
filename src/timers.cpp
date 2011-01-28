/*
 *  timers.c
 *  sim
 */
#include <ctime>
#include "timers.h"
using namespace robot;

#ifdef REAL_ROBOT

unsigned long robot::micro_time(void) {
  return clock() * 1000000 / CLOCKS_PER_SEC;
}

unsigned long robot::milli_time(void) {
  return clock() * 1000 / CLOCKS_PER_SEC;
}

double robot::time(void) {
  return ((double)clock()) / ((double)CLOCKS_PER_SEC);
}

#else

#include "simulation.h"

sim::Simulation * sim::curSim = NULL;

unsigned long robot::micro_time(void) {
  return sim::curSim->time * 1000000;
}

unsigned long robot::milli_time(void) {
  return sim::curSim->time * 1000;
}

double robot::time(void) {
  return sim::curSim->time;
}

#endif