/*
 *  simulation.h
 *  sim
 */

#include "geometry.h"
#include "map.h"
#include "timers.h"

#ifndef __ROBOT_H__
namespace robot {
  class AbstractRobot;
}
#endif

#ifndef __SIMULATION_H__
#define __SIMULATION_H__

namespace sim {
  
  class Robot {
  public:
    // the following are the true values, not where the bot thinks it is
    // position in cm from the lower left of the house
    math::Ray lastPosition;
    math::Ray position;
    // velocity in cm / s
    math::vec2 velocity;
    // angular velocity in radians / sec
    float angularVelocity;
    
    Robot() : position(), velocity(), angularVelocity(0) {}
  };
  
  class World {
  public:
    sim::Map * map;
    const math::Ray& position;
    robot::AbstractRobot * bot;
    
    World(const math::Ray& pos, robot::AbstractRobot * b = NULL)
      : map(NULL), position(pos), bot(b) { }
    virtual ~World();
    
    virtual void step() = 0;
    
    virtual bool realTime() = 0;
    virtual void start() = 0;
    virtual void pause();
  };
  
  class Simulation : public World {
    public:
    sim::Robot simBot;
    math::vec2 candle;
    float time;
    float deltaT;
    
    Simulation()
      : World(simBot.position), simBot(), candle(), time(0.0), deltaT(0.1)
    {}
    
    virtual void step();  // do one iteration through the simulation
    virtual bool realTime() {
      return false;
    }
    virtual void start() {}
  };
  
  class RealTimeSimulation : public Simulation {
  protected:
    struct timeval lastTime;
  public:
    RealTimeSimulation() : Simulation(), lastTime() {}
    virtual void step();
    virtual bool realTime() {
      return true;
    }
    virtual void start();
  };
  
  class RealWorld : public World {
  public:
    explicit RealWorld(robot::AbstractRobot * b);
    
    virtual void step();
    
    virtual bool realTime() {
      return true;
    }
    virtual void start() { };
  };
  
  Simulation * create_simulation(robot::AbstractRobot * bot, const char * mapPath,
                                const char * botPath, const char * sensLibPath);
  
} // namespace sim
#endif // __SIMULATION_H__
