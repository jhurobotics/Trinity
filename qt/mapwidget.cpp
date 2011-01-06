#include "mapwidget.h"
#include "../src/map.h"

MapWidget::MapWidget(QWidget *parent) :
    QGLWidget(parent), theMap(NULL)
{
}

MapWidget::~MapWidget() {
  if( theMap ) {
    delete theMap;
  }
}


void MapWidget::setMap(sim::Map *newMap) {
  if( theMap && theMap != newMap ) {
    delete theMap;
  }
  theMap = newMap;
  update();
}

void MapWidget::initializeGL()
{
  glClearColor(1.0, 1.0, 1.0, 0.0);
}

void MapWidget::resizeGL(int width, int height) {
  glViewport(0, 0, width, height);
  paintGL();
}

void MapWidget::paintGL()
{
  glClear(GL_COLOR_BUFFER_BIT);

  if( !theMap ) {
    return;
  }

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho( -5.0 , theMap->width+5.0, -5.0, theMap->height+5.0, -1.0, 1.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  sim::draw_map(theMap, true, true);
  glFlush();
}
