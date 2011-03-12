//
//  robotwidget.cpp
//  sim
//

#include "robotwidget.h"
#include "../src/simulation.h"
#include "../src/robot.h"
using namespace sim;

#define RED 1.0, 0.0, 0.0
#define GREEN 0.0, 1.0, 0.0
#define BLUE 0.0, 0.0, 1.0
#define WHITE 1.0, 1.0, 1.0

RobotWidget::RobotWidget(QWidget *parent) :
QGLWidget(parent), world(NULL), subThread(world, mutex)
{
  refreshTimer.setInterval(500);
  connect(&refreshTimer, SIGNAL(timeout()), this, SLOT(update()));
}

RobotWidget::~RobotWidget() {
  if( world ) {
    delete world;
  }
}

void RobotWidget::setWorld(sim::World *newWorld) {
  if( world && newWorld != world ) {
    delete world;
  }
  world = newWorld;
  update();
}

void RobotWidget::initializeGL() {
  glClearColor(1.0, 1.0, 1.0, 0.0);
}

void RobotWidget::resizeGL(int width, int height) {
  glViewport(0, 0, width, height);
  paintGL();
}

void RobotWidget::paintGL() {
  mutex.lock();
  glClear(GL_COLOR_BUFFER_BIT);
  
  if( !world ) {
    return;
  }
  
  
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho( - 25.0 , world->map->width + 25.0,
          - 25.0, world->map->height + 25.0,
          -1.0, 1.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glPointSize(1.0);
  
  sim::draw_map(world->map, true, true);
  
  // fade the map
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glColor4f(WHITE, 0.5f);
  glRectf(0, 0, world->map->width, world->map->height);
  
  glPushMatrix();
  glTranslatef(world->position.origin().x, world->position.origin().y, 0.0);
  glRotatef((atan2(world->position.dir().y, world->position.dir().x))*180.0/M_PI, 0.0, 0.0, 1.0);
  glColor4f(BLUE, 1.0);
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
  
  glPushMatrix();
  world->bot->draw();
  glPopMatrix();
  
  glFlush();
  mutex.unlock();
}

void RobotWidget::stepOnce() {
  executeSteps(1);
}

void RobotWidget::stepSecond() {
  executeSteps(10);
}

void RobotWidget::executeSteps(unsigned int count) {
  if( world ) {
    for( unsigned int i = 0; i < count; i++ ) {
      world->step();
    }
    update();
  }
}

void RobotWidget::start() {
  world->start();
  subThread.go = true;
  subThread.start();
  refreshTimer.start();
}

void RobotWidget::step() {
  world->step();
  update();
}

void RobotWidget::stop() {
  refreshTimer.stop();
  subThread.go = false;
  subThread.wait();
  update();
}

void RobotWidget::RobotThread::run() {
  while(go) {
    mutex.lock();
    theWorld->step();
    mutex.unlock();
  }
}