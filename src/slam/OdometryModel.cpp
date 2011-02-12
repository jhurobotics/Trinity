/*
 *  OdometryModel.cpp
 *  sim
 */

#include "slam.h"
using namespace robot;

static float sample(float input) {
  return 0.0;
}

Pose MCL::sample_motion_model_odometry(const Odometry& u_t, const Pose& lastPose) {
  const math::vec2& x_bar = u_t.prev.origin();
  const math::vec2& x_bar_prime = u_t.next.origin();
  float dx = x_bar.x - x_bar_prime.x;
  float dy = x_bar.y - x_bar_prime.y;
  
  // On the first and third angles, the book subtracts the angle of the last pose
  // But, the odometry data is in the coordinate system of the last pose, so that
  // would just cancel everything out...
  float delta_rot1 = atan2( x_bar_prime.y - x_bar.y, x_bar_prime.x - x_bar.x);
  float delta_trans = sqrt( dx * dx + dy * dy );
  float delta_rot2 = u_t.next.angle() - u_t.prev.angle();
    
  float delta_rot1_hat = delta_rot1 - sample(odometryNoise[0]*delta_rot1 + odometryNoise[1]*delta_trans);
  float delta_trans_hat = delta_trans - sample(odometryNoise[2]*delta_trans + odometryNoise[3]*(delta_rot1 + delta_rot2));
  float delta_rot2_hat = delta_rot2 - sample(odometryNoise[0]*delta_rot2 + odometryNoise[1]*delta_trans);
  
  math::vec2 finalPos( lastPose.origin().x + delta_trans_hat*cos(lastPose.angle() + delta_rot1_hat),
                       lastPose.origin().y + delta_trans_hat*sin(lastPose.angle() + delta_rot1_hat)
                      );
  float finalHeading = lastPose.angle() + delta_rot1_hat + delta_rot2_hat;
  return Pose(finalPos, finalHeading);
}