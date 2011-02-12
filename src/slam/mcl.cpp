/*
 *  mcl.cpp
 */
#include <assert.h>
#include "../math.h"
#include "slam.h"
#include "../robot.h"
using namespace robot;
using namespace math;

#define PARTICLE_COUNT  100

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

void MCL::initialize(Pose start, float range, const sim::Map& m) {
  bels[0].clear();
  bels[0].reserve(PARTICLE_COUNT);
  bels[1].clear();
  cur_bel = 0;
  for( unsigned int i = 0; i < PARTICLE_COUNT; i++ ) {
    float x = randFloat(range, start.origin().x);
    float y = randFloat(range, start.origin().y);
    float angle = randFloat(M_PI, start.angle());
    bels[0].push_back(Pose(vec2(x,y), angle));
  }
  lastPose = start;
  lastCount[0] = lastCount[1] = 0;
  map = m;
}

void MCL::low_variance_sampler(const weighted_belief_t & input, float total,
                                 belief_t * output) {
  assert(output);
  output->reserve(input.size());
  
  float M_inverse = 1.0 / ((float) input.size());
  float r = math::randFloat(M_inverse/2.0, M_inverse/2.0);
  float c = input[0].second;
  unsigned int i = 0;
  for( unsigned int m = 0; m < input.size(); m++ ) {
    float u = (r + ((float)m) * M_inverse) * total;
    while( u > c && u < total ) {
      i++;
      c += input[i].second;
    }
    output->push_back(input[i].first);
  }
}

void MCL::mcl( const belief_t& last_bel, const Odometry& u, const Measurements& z,
         belief_t * new_bel)
{
  assert(new_bel);
  unsigned int M = last_bel.size();
  weighted_belief_t bel_bar;
  bel_bar.reserve(M);
  float totalWeight = 0.0;
  for( unsigned int m = 0; m < M; m++ ) {
    Pose x = sample_motion_model_odometry(u, last_bel[m]);
    float w = sample_measurement_model(z, x);
    bel_bar.push_back(std::pair<Pose, float>(x, w));
    totalWeight += w;
  }
  low_variance_sampler(bel_bar, totalWeight, new_bel);
}

Pose MCL::getAverage(const belief_t& bel) {
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

Pose MCL::determineNext(Pose curPose)
{
  unsigned long newCount = encoders[0]->getCount();
  float s1 = ((float)newCount - lastCount[0]) * encoders[0]->tickDist; // left
  lastCount[0] = newCount;
  newCount = encoders[1]->getCount();
  float s2 = ((float)newCount - lastCount[1]) * encoders[1]->tickDist; // right
  lastCount[1] = newCount;
  float d = (encoders[0]->relPos - encoders[1]->relPos).mag();
  // assume that we're turning left
  float theta = (s2-s1)/d;
  
  vec2 origin;
  float angle;
  if( fabsf(theta) < 0.05 ) {
    origin = curPose.getPoint((s1+s2)/2);
    angle = curPose.angle()+theta;
    return Ray(origin, angle);
  }
  
  float r = s1 * d / (s2 - 21);
  origin = (encoders[1]->relPos - encoders[0]->relPos)*r + curPose.origin();
  float R = r + d / 2;
  vec2 disp;
  disp.x = ( 1 - cos(theta) ) * R;
  disp.y = sin(theta) * R;
  angle = curPose.angle() + theta;
  return Ray( curPose.origin() + disp, angle);
}

Pose MCL::getPose() {
  belief_t * belief;
  belief_t * next_belief;
  if( cur_bel == 0 ) {
    belief = &(bels[0]);
    next_belief = &(bels[1]);
  }
  else {
    belief = &(bels[1]);
    next_belief = &(bels[0]);
  }
  next_belief->clear();
  // get the sensor / odometry data somehow
  
  Odometry u;
  Pose nextPose = determineNext(lastPose);
  u.prev = lastPose;
  u.next = nextPose;
  lastPose = nextPose;

  Measurements z;
  mcl(*belief, u, z, next_belief);
  cur_bel ++;
  cur_bel %= 2;
  return getAverage(*next_belief);
}

#ifndef __APPLE__
#include <GL/gl.h>
#else
#include <OpenGL/gl.h>
#endif

#define RED 1.0, 0.0, 0.0

void MCL::draw() {
  const belief_t& bel = ( cur_bel ? bels[1] : bels[0] );
  glColor4f(RED, 1.0);
  
  belief_t::const_iterator end = bel.end();
  for(belief_t::const_iterator iter = bel.begin(); iter != end; iter++ ) {
    glPushMatrix();
    glTranslatef(iter->origin().x, iter->origin().y, 0.0);
    glRotatef((atan2(iter->dir().y, iter->dir().x))*180.0*M_1_PI, 0.0, 0.0, 1.0);
    glScalef(0.4, 0.4, 0.4);
    glBegin(GL_LINE_LOOP);
    int angleCount = 32;
    for(int i = 0; i < angleCount; i++) {
      glVertex2f(4*cos(((float)i)/((float)angleCount)*2*M_PI), 4*sin(((float)i)/((float)angleCount)*2*M_PI));
    }
    glEnd();
    glBegin(GL_LINES);
    glVertex2f(0.0, 0.0);
    glVertex2f(8.0, 0.0);
    glVertex2f(6.0, -2.0);
    glVertex2f(8.0, 0.0);
    glVertex2f(8.0, 0.0);
    glVertex2f(6.0, 2.0);
    glEnd();
    
    glPopMatrix();
  }
}
