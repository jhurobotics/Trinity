#ifndef MAPWIDGET_H
#define MAPWIDGET_H

#include <QGLWidget>

#ifndef __MAP_H__
namespace sim {
  class Map;
}
#endif // __MAP_H__

class MapWidget : public QGLWidget
{
  Q_OBJECT
protected:
  sim::Map * theMap;
public:
  explicit MapWidget(QWidget *parent = 0);
  virtual ~MapWidget();

protected:
  virtual void initializeGL();
  virtual void resizeGL(int width, int height);
  virtual void paintGL();
signals:

public slots:
  void setMap(sim::Map * newMap);

};

#endif // MAPWIDGET_H
