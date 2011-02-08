/*
 *  mcl.cpp
 */
#include <assert.h>
#include "slam.h"
using namespace robot;
using namespace math;

namespace robot {
  void low_variance_sampler(const weighted_belief_t & input, float total,
                                   belief_t * output);
}

void robot::low_variance_sampler(const weighted_belief_t & input, float total,
                                 belief_t * output) {
  assert(output);
  output->reserve(input.size());
  
  float M_inverse = 1.0 / ((float) input.size());
  float r = math::randFloat(M_inverse/2.0, M_inverse/2.0);
  float c = input[0].second;
  unsigned int i = 0;
  for( unsigned int m = 0; m < input.size(); m++ ) {
    float u = r + m * M_inverse;
    while( u > c && u < 1.0 ) {
      i++;
      c += input[i].second / total;
    }
    output->push_back(input[i].first);
  }
}

void robot::mcl( const belief_t& last_bel, const Odometry& u, const Measurements& z,
         const MeasurementMap map, belief_t * new_bel)
{
  assert(new_bel);
  unsigned int M = last_bel.size();
  weighted_belief_t bel_bar;
  bel_bar.reserve(M);
  float totalWeight = 0.0;
  for( unsigned int m = 0; m < M; m++ ) {
    Pose x = sample_motion_model_odometry(u, last_bel[m]);
    float w = sample_measurement_model(z, x, map);
    bel_bar.push_back(std::pair<Pose, float>(x, w));
    totalWeight += w;
  }
  low_variance_sampler(bel_bar, totalWeight, new_bel);
}

Pose robot::getAverage(const belief_t& bel) {
  vec2 pos(0,0);
  vec2 dir(0,0);
  unsigned int count = 0;
  belief_t::const_iterator end = bel.end();
  for( belief_t::const_iterator iter = bel.begin(); iter != end; iter++ ) {
    pos += iter->origin();
    dir += iter->dir();
    count ++;
  }
  return Pose( pos/((float)count), dir/((float)count));
}