#include <boost/python.hpp>
#include "../math.h"
#include "../geometry.h"
using namespace boost::python;
using namespace math;

BOOST_PYTHON_MODULE(math)
{
  def("randFloat", randFloat);
  
  // does not expose internal data!
  class_<mat2>("mat2")
    .def(init<float, float, float, float>())
    .def("det", &mat2::det)
    .def(self * vec2())
  ;
  
  def("getRotationMatrix", getRotationMatrix);
  
  class_<vec2>("vec2")
    .def(init<float, float>())
    .def(init<const vec2&>())
    .def_readwrite("x", &vec2::x)
    .def_readwrite("y", &vec2::y)
    .def("mag_sq", &vec2::mag_sq)
    .def("mag", &vec2::mag)
    .def(self + vec2())
    .def(self - vec2())
    .def("dot", &vec2::dot)
    .def(self * float())
    .def(self / float())
    //.def(self = vec2())
    .def(self += vec2())
    .def(self -= vec2())
    .def(self *= float())
    .def(self /= float())
    .def("normalize", &vec2::normalize, return_internal_reference<1>())
    .def("cross", &vec2::cross)
    .def("transpose", &vec2::transpose)
    .def(self == vec2())
    .def(self != vec2())
    .def("getRotationMatrix", &vec2::getRotationMatrix)
    .def(-self)
    .def(float() * self)
    .def(float() / self)
    .def(self *= mat2())
  ;
  
  class_<Segment>("Segment")
    .def(init<const vec2&, const vec2&>())
    .def(init<const Segment&>())
    .def("direction", &Segment::direction)
    //.def_readwrite("start", &Segment::start)  These are protected, and accessed through []
    //.def_readwrite("end", &Segment::end)
  ;
  
  class_<Ray>("Ray")
    .def(init<const vec2&, const vec2&>())
    .def(init<const Ray&>())
    .def("origin", &Ray::origin, return_internal_reference<>())
    .def("setOrigin", &Ray::setOrigin) //.add_property("origin", &Ray::origin, &Ray::setOrigin)
    .def("dir", &Ray::dir, return_internal_reference<>())
    .def("setDir", &Ray::setDir)  //.add_property("dir", &Ray::dir, &Ray::setDir)
    .def("getPoint", &Ray::getPoint)
    .def("transform", &Ray::transform, return_internal_reference<>())
    .def("rotateAboutStart", &Ray::rotateAboutStart, return_internal_reference<>())
    .def(self += vec2())
    .def(self += Ray())
    .def("transformRayToAbsolute", &Ray::transformRayToAbsolute)
    .def("transformVecToAbsolute", &Ray::transformVecToAbsolute)
    .def("transformToLocal", &Ray::transformToLocal)
    //.def(self = Ray(), return_internal_reference))
  ;
  
  class_<Circle>("Circle", init<const vec2&, float>())
    .def_readwrite("center", &Circle::center)
    //.add_property("center", &Circle::center, &Circle::setCenter)
    .add_property("radius", &Circle::radius, &Circle::setRadius)
    .def("contains", &Circle::contains)
  ;
  
  def("distance", &distance);
  def("pointToSeg", &pointToSeg);
  def("intersect", &intersect);
}
