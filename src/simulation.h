/*
 *  simulation.h
 *  sim
 */

#ifndef __SIMULATION_H__
#define __SIMULATION_H__

#include "geometry.h"
#include "map.h"

#ifndef __ROBOT_H__
namespace robot {
  class AbstractRobot;
}
#endif

namespace sim {
  
  class Robot {
  public:
    robot::AbstractRobot * bot;
    
    // the following are the true values, not where the bot thinks it is
    // position in cm from the lower left of the house
    math::Ray position;
    // velocity in cm / s
    math::vec2 velocity;
    // angular velocity in radians / sec
    float angularVelocity;
  };
  
  class Simulation {
    public:
    sim::Robot bot;
    sim::Map * map;
    math::vec2 candle;
    float time;
    float deltaT;
    
    Simulation()
      : bot(), map(NULL), candle(), time(0.0), deltaT(0.1)
    {}
    
    void step();  // do one iteration through the simulation
  };
  
  Simulation * create_simulation(const char * mapPath,
                                const char * botPath, const char * sensLibPath);
  
} // namespace sim
#endif // __SIMULATION_H__