#include "simwidget.h"
#include "../src/simulation.h"
#include "../src/robot.h"
using namespace sim;

#define RED 1.0, 0.0, 0.0
#define GREEN 0.0, 1.0, 0.0
#define BLUE 0.0, 0.0, 1.0
#define WHITE 1.0, 1.0, 1.0

SimWidget::SimWidget(QWidget *parent) :
    QGLWidget(parent), world(NULL)
{
}

SimWidget::~SimWidget() {
  if( world ) {
    delete world;
  }
}

void SimWidget::setWorld(sim::Simulation *newWorld) {
  if( world && newWorld != world ) {
    delete world;
  }
  world = newWorld;
  update();
}

void SimWidget::stepOnce() {
  executeSteps(1);
}

void SimWidget::stepSecond() {
  executeSteps(10);
}

void SimWidget::executeSteps(unsigned int count) {
  if( world ) {
    for( unsigned int i = 0; i < count; i++ ) {
      world->step();
    }
    update();
  }
}

void SimWidget::initializeGL() {
  glClearColor(1.0, 1.0, 1.0, 0.0);
}

void SimWidget::resizeGL(int width, int height) {
  glViewport(0, 0, width, height);
  paintGL();
}

void SimWidget::paintGL() {
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
  
  robot::Robot * bot = dynamic_cast<robot::Robot*>(world->bot.bot);
  if( bot ) {
    const std::vector<math::vec2>& path = bot->path;
    const std::vector<math::vec2>& edges = bot->edges;
    const std::vector<math::vec2>& realPoints = bot->realPoints;
  
    glPointSize(4.0);
    glBegin(GL_POINTS);
    glColor4f(RED, 1.0);
    for( unsigned int i = 0; i < realPoints.size(); i++ ) {
      glVertex2f(realPoints[i].x, realPoints[i].y);
    }
    glEnd();
    
    glPushMatrix();
    glTranslatef(world->map->start.x, world->map->start.y, 0);
    glRotatef(-90.0, 0.0, 0.0, 1.0);
    glPointSize(4.0);
    glColor4f(BLUE, 1.0);
    glBegin(GL_POINTS);
    for( unsigned int i = 0; i < path.size(); i++ ) {
      glVertex2f(path[i].x, path[i].y);
    }
    glColor4f(GREEN, 1.0);
    for( unsigned int i = 0; i < edges.size(); i++ ) {
      glVertex2f(edges[i].x, edges[i].y);
    }
    glEnd();
    glPopMatrix();
  }    
  
  glPushMatrix();
  glTranslatef(world->bot.position.origin().x, world->bot.position.origin().y, 0.0);
  glRotatef((atan2(world->bot.position.dir().y, world->bot.position.dir().x))*180.0/M_PI, 0.0, 0.0, 1.0);
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

  glFlush();
}
