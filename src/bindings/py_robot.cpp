#include <boost/python.hpp>
#include "../robot.h"
#include "../simulation.h"
#include "../simsensors.h"
using namespace boost::python;
using namespace robot;

class Python_RangeSensor : public RangeSensor, wrapper<RangeSensor> {
  public:
  Python_RangeSensor(PyObject* self, const RangeSpecs& s) : RangeSensor(s), m_self(self) {}
  
  virtual float getValue() {
    return call_method<float>(m_self, "getValue");
  }
  
  private:
  PyObject* const m_self;
};

class Python_SensorFactory : public SensorFactory, wrapper<SensorFactory> {
  public:
  Python_SensorFactory(PyObject* self) : m_self(self) {}
  
  virtual RangeSensor * rangeSensor(const std::string& name) {
    return call_method<RangeSensor*>(m_self, "rangeSensor", name);
  }
  
  private:
  PyObject* const m_self;
};

class Python_MotorControl : public MotorControl, wrapper<MotorControl> {
  private:
  PyObject* const m_self;
  public:
  Python_MotorControl(PyObject* self) : m_self(self) {}
  
  virtual void setVelocity(float velocity) {
    call_method<void, float>(m_self, "setVelocity", velocity);
  }
  
  virtual void setAngularVelocity(float angVel) {
    call_method<void, float>(m_self, "setAngularVelocity", angVel);
  }
};

class Python_MotorFactory : public MotorFactory, wrapper<MotorFactory> {
  private:
  PyObject* const m_self;
  public:
  Python_MotorFactory(PyObject* self) : m_self(self) {}
  
  virtual MotorControl * newMotors() {
    return call_method<MotorControl*>(m_self, "newMotors");
  }
};

class Python_AbstractRobot : public AbstractRobot, wrapper<AbstractRobot> {
  private:
  PyObject* const m_self;
  public:
  Python_AbstractRobot(PyObject* self) : m_self(self) {}
  
  virtual void act() {
    call_method<void>(m_self, "act");
  }

  virtual void addRangeSensor(RangeSensor *sensor) {
    call_method<void>(m_self, "addRangeSensor", sensor);
  }

  virtual void addMotors(MotorControl *motors) {
    call_method<void>(m_self, "addMotors", motors);
  }

  virtual void draw() {
    if( override f = this->get_override("draw") ) {
      f();
    }
    else {
      AbstractRobot::draw();
    }
  }
};

BOOST_PYTHON_MODULE(robot) {
  
  class_<RangeSpecs>("RangeSpecs")
    .def_readwrite("maxRange", &RangeSpecs::maxRange)
    .def_readwrite("minRange", &RangeSpecs::minRange)
    .def_readwrite("error", &RangeSpecs::error)
    .def_readwrite("tanOfWidth", &RangeSpecs::tanOfWidth)
  ;
  
  class_<RangeSensor, Python_RangeSensor, boost::noncopyable>("RangeSensor", init<const RangeSpecs&>())
    .def("getValue", pure_virtual(&RangeSensor::getValue))
    .def("getCoordinate", &RangeSensor::getCoordinate)
  ;
  
  class_<SensorFactory, Python_SensorFactory, boost::noncopyable>("SensorFactory")
    .def("rangeSensor", pure_virtual(&SensorFactory::rangeSensor), return_value_policy<manage_new_object>())
  ;
  
  class_<MotorControl, Python_MotorControl, boost::noncopyable>("MotorControl")
    .def("setVelocity", pure_virtual(&MotorControl::setVelocity))
    .def("setAngularVelocity", pure_virtual(&MotorControl::setAngularVelocity))
  ;
  
  class_<MotorFactory, Python_MotorFactory, boost::noncopyable>("MotorFactory")
    .def("newMotors", pure_virtual(&MotorFactory::newMotors), return_value_policy<manage_new_object>())
  ;
  
  class_<AbstractRobot, Python_AbstractRobot, boost::noncopyable>("AbstractRobot")
    .def("act", pure_virtual(&AbstractRobot::act))
    .def("addRangeSensor", pure_virtual(&AbstractRobot::addRangeSensor))
    .def("addMotors", pure_virtual(&AbstractRobot::addMotors))
    .def("draw", &AbstractRobot::draw, &Python_AbstractRobot::draw)
  ;
}
