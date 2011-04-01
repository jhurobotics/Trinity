/*#include "../qt/mainwindow.h"
#include "../qt/ui_mainwindow.h"
#include "../qt/mapwidget.h"*/
#include "map.h"
//#include "../qt/robotwidget.h"
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
  if(argc != 3)
  {
    puts("Invalid number of arguments, dying now.");
    return -1;
  }
  std::string robotPath = argv[1], sensorPath = argv[2], mapPath = argv[3];
  robot::AbstractRobot * bot = robot::new_robot(robot::SONAR);
  setupflags_t setupflags = 0;
  setupflags |= MOTORS_MAESTRO;
  setupflags |= SONAR_ARDUINO;
  setupflags |= ENCODER_ARDUINO;
  sim::World * world = create_world(bot, mapPath.c_str(),
                                    robotPath.c_str(), sensorPath.c_str(),
                                    setupflags);
  return 0;
}
