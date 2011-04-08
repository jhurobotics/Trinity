#include "map.h"
#include "simulation.h"
#include "robot.h"
#include "setup.h"
#include <string>
#include <cstdio>
using namespace std;
using namespace sim;
/*
Need path to robot, path to sensors, path to map
*/
int main(int argc, char** argv)
{
  if(argc < 5)
  {
    printf("cli:\n");
    printf("cli robotPath sensorPath arduinoPath maestroPath [--speedTest]\n");
    puts("Invalid number of arguments, dying now.");
    return -1;
  }
  std::string robotPath = argv[1], sensorPath = argv[2];//, mapPath = argv[3];
  const char * arduino = argv[3];
  const char * maestro = argv[4];
  robot::Implementation imp = robot::SONAR;
  if( argc == 6 ) {
    std::string test = argv[5];
    if( test == "--speedtest" ) {
      imp = robot::SPEEDTEST;
    } 
  }
  robot::AbstractRobot * bot = robot::new_robot(imp);
  setupflags_t setupflags = 0;
  setupflags |= MOTORS_MAESTRO;
  setupflags |= SONAR_ARDUINO;
  setupflags |= ENCODER_ARDUINO;
  sim::World * world = create_world(bot, NULL, //mapPath.c_str(),
                                    robotPath.c_str(), sensorPath.c_str(),
                                    setupflags, arduino, maestro);
  for(;;) {
    world->step();
  }
  
  delete world;
  return 0;
}
