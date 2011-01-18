/*
 *  OdometryModel.cpp
 *  sim
 */

#include "slam.h"
using namespace robot;

float sample(float input) {
  return 0.0;
}

// Algorithm from Table 5.5 (p. 108) in Probabilistic Robotics 
Pose sample_motion_model_odometry(const Odometry& u_t, const Pose& lastPose) {
  const math::vec2& x_bar = u_t.prev.origin();
  const math::vec2& x_bar_prime = u_t.next.origin();
  float dx = x_bar_prime.x - x_bar.x;
  float dy = x_bar_prime.y - x_bar.y;
  
  float delta_rot1 = atan2( x_bar_prime.y - x_bar.y, x_bar_prime.x - x_bar.x) - lastPose.angle();
  float delta_trans = sqrt( dx * dx + dy * dy );
  float delta_rot2 = u_t.next.angle() - u_t.prev.angle() - delta_rot1;
  
  float delta_rot1_hat = delta_rot1 - sample(odometryNoise[0]*delta_rot1 + odometryNoise[1]*delta_trans);
  float delta_trans_hat = delta_trans - sample(odometryNoise[2]*delta_trans + odometryNoise[3]*(delta_rot1 + delta_rot2));
  float delta_rot2_hat = delta_rot2 - sample(odometryNoise[0]*delta_rot2 + odometryNoise[1]*delta_trans);
  
  math::vec2 finalPos( lastPose.origin().x + delta_trans_hat*cos(lastPose.angle() + delta_rot1_hat),
                       lastPose.origin().y + delta_trans_hat*sin(lastPose.angle() + delta_rot1_hat)
                      );
  float finalHeading = lastPose.angle() + delta_rot1_hat + delta_rot2_hat;
  return Pose(finalPos, finalHeading);
}