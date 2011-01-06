#include "mapwidget.h"
#include "../src/map.h"
#include <iostream>

MapWidget::MapWidget(QWidget *parent) :
    QGLWidget(parent), theMap(NULL)
{
  std::cout << "created a MapWidget\n";
}

MapWidget::~MapWidget() {
  if( theMap ) {
    delete theMap;
  }
}


void MapWidget::setMap(sim::Map *newMap) {
  if( theMap ) {
    delete theMap;
  }
  theMap = newMap;
}

void MapWidget::initializeGL()
{
  glClearColor(1.0, 1.0, 1.0, 0.0);
  std::cout << "initialized OpenGL\n";
}

void MapWidget::resizeGL() {
  const QSize size = frameSize();
  glViewport(0, 0, size.width(), size.height());

  if( !theMap ) {
    return;
  }

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho( -5.0 , theMap->width+5.0, -5.0, theMap->height+5.0, -1.0, 1.0);
  std::cout << "map dimensions: " << theMap->width << " x " << theMap->height << "\n";
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
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
  std::cout << "map dimensions: " << theMap->width << " x " << theMap->height << "\n";
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  sim::draw_map(theMap, true, true);
  glFlush();
}
