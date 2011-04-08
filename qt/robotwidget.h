/* -*-C++-*- */
//  robotwidget.h
//  sim
//

#ifndef ROBOTWIDGET_H
#define ROBOTWIDGET_H

#include <QGLWidget>
#include <QMutex>
#include <QThread>
#include <QTimer>

namespace sim {
  
#ifndef __SIMULATION_H__
  class World;
#endif
  
  class RobotWidget : public QGLWidget
  {
    Q_OBJECT
  public:
    explicit RobotWidget(QWidget *parent = 0);
    virtual ~RobotWidget();
    
  protected:
    sim::World * world;
    
    class RobotThread : public QThread {
    public:
      sim::World * &theWorld;
      QMutex &mutex;
      volatile bool go;
      RobotThread(sim::World *& w, QMutex &m)
      : theWorld(w), mutex(m), go(true) {}
      virtual void run();
    };
    
    RobotThread subThread;
    QTimer refreshTimer;
    QMutex mutex;
    
    
    virtual void initializeGL();
    virtual void resizeGL(int width, int height);
    virtual void paintGL();
    
    void executeSteps(unsigned int count);
    
    public slots:
    void setWorld(sim::World * newWorld);
    
    void stepOnce();
    void stepSecond();
    
    void start();
    void stop();
    void step();
  };
  
} // namespace sim
#endif // ROBOTWIDGET_H
