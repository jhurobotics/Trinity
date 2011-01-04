/*
 *  simmotors.cpp
 *  sim
 */

#include "simmotors.h"
using namespace sim;

void sim::MotorControl::setVelocity(float speed) {
  bot->velocity = bot->position.dir() * speed;
}

void sim::MotorControl::setAngularVelocity(float angVel) {
  bot->angularVelocity = angVel;
}

robot::MotorControl* sim::MotorFactory::newMotors() {
  return new sim::MotorControl(world);
}
