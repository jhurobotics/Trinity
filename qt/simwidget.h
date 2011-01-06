#ifndef SIMWIDGET_H
#define SIMWIDGET_H

#include <QGLWidget>

#ifndef __SIMULATION_H__
namespace sim {
  class Simulation;
}
#endif // __SIMULATION_H__

class SimWidget : public QGLWidget
{
    Q_OBJECT
public:
    explicit SimWidget(QWidget *parent = 0);
    virtual ~SimWidget();

protected:
    sim::Simulation * world;

    void executeSteps(unsigned int count);

    virtual void initializeGL();
    virtual void resizeGL(int width, int height);
    virtual void paintGL();

public slots:
  void stepOnce();
  void stepSecond();
  void setWorld(sim::Simulation * newWorld);
};

#endif // SIMWIDGET_H
