/*
 *  geometry.h
 *  sim
 */

#include <cmath>
#include <cstring>

#ifndef __GEOMETRY_H__
#define __GEOMETRY_H__

namespace math {
  
  class InvalidIndexException {};
  
  class mat2 {
    public:
    // [row][col]
    float data[2][2];
    
    explicit mat2() throw() {
      data[0][0] = data[1][1] = 1;
      data[0][1] = data[1][0] = 0;
    }
    explicit mat2(float d11, float d12, float d21, float d22) throw() {
      data[0][0] = d11;
      data[0][1] = d12;
      data[1][0] = d21;
      data[1][1] = d22;
    }
    
    const float* operator[](int idx) const throw(InvalidIndexException) {
      switch(idx) {
        case 0:
          return data[0];
        case 1:
          return data[1];
        default:
          throw InvalidIndexException();
      }
    }
    
    float det() const throw() {
      return data[0][0] * data[1][1] - data[0][1] * data[1][0];
    }
    
    mat2 operator*(const mat2& other) throw() {
      mat2 result;
      result.data[0][0] = data[0][0]*other.data[0][0] + data[0][1]*other.data[1][0];
      result.data[0][1] = data[0][0]*other.data[0][1] + data[0][1]*other.data[1][1];
      result.data[1][0] = data[1][0]*other.data[0][0] + data[1][1]*other.data[1][0];
      result.data[1][1] = data[1][0]*other.data[0][1] + data[1][1]*other.data[1][1];
      return result;
    }
  }; // class mat2
  
  mat2 getRotationMatrix(float angle) throw();
    
  class vec2 {
    public:
    float x;
    float y;
    
    explicit vec2() throw() : x(0), y(0) {}
    explicit vec2(float nx, float ny) throw() : x(nx), y(ny) {}
    vec2(const vec2& other) throw() : x(other.x), y(other.y) {}
    
    float& operator[](int idx) throw(InvalidIndexException) {
      switch(idx) {
        case 0:
          return x;
        case 1:
          return y;
        default:
          throw InvalidIndexException();
      }
    }
    
    float operator[](int idx) const throw(InvalidIndexException) {
      switch(idx) {
        case 0:
          return x;
        case 1:
          return y;
        default:
          throw InvalidIndexException();
      }
    }
    
    float mag_sq() const throw() {
      return x * x + y * y;
    }
    
    float mag() const throw() {
      return sqrt(mag_sq());
    }
    
    vec2 operator+(const vec2& other) const throw() {
      return vec2( x + other.x, y + other.y );
    }
    vec2 operator-(const vec2& other) const throw() {
      return vec2( x - other.x, y - other.y );
    }
    float dot(const vec2& other) const throw() {
      return x * other.x + y * other.y;
    }
    vec2 operator*(float scale) const throw() {
      return vec2( x * scale, y * scale );
    }
    vec2 operator/(float scale) const throw() {
      return vec2( x / scale, y / scale );
    }
    vec2& operator=(const vec2& other) throw() {
      x = other.x;
      y = other.y;
      return *this;
    }
    vec2& operator+=(const vec2& other) throw() {
      x += other.x;
      y += other.y;
      return *this;
    }
    vec2& operator-=(const vec2& other) throw() {
      x -= other.x;
      y -= other.y;
      return *this;
    }
    vec2& operator*=(float scale) throw() {
      x *= scale;
      y *= scale;
      return *this;
    }
    vec2& operator/=(float scale) throw() {
      x /= scale;
      y /= scale;
      return *this;
    }
    float& operator[](unsigned int idx) throw(InvalidIndexException) {
      switch(idx) {
        case 0:
          return x;
        case 1:
          return y;
        default:
          throw InvalidIndexException();
      }
    }
    vec2& normalize() throw() {
      return (*this) /= mag();
    }
    float cross(const vec2& other) const throw() {
      return x*other.y - y*other.x;
    }
    
    vec2 transpose() const throw() {
      return vec2(y, x);
    }
        
    bool operator==(const vec2& other) const throw() {
      return x == other.x && y == other.y;
    }
    bool operator!=(const vec2& other) const throw() {
      return x != other.x || y != other.y;
    }
    
    // Assuming this vector is normalized
    // Get the rotation associated with this forward vector
    mat2 getRotationMatrix() const throw() {
      return mat2( x, -y, y, x);
    }
    
    // for std::less, doesn't actually mean anything
    bool operator<(const vec2 other) const throw() {
      if( x != other.x ) {
        return x < other.x;
      }
      else {
        return y < other.y;
      }
    }
    
  }; // class vec2
  
  static inline vec2 operator-(const vec2& arg) throw() {
    return vec2(-(arg[0]), -(arg[1]));
  }
  static inline vec2 operator*(float scalar, const vec2& vector) throw() {
    return vec2(vector[0] * scalar, vector[1] * scalar);
  }
  static inline vec2 operator/(float scalar, const vec2& vector) throw() {
    return vec2(scalar / (vector[0]), scalar / (vector[1]) );
  }
  static inline vec2 operator*(const mat2& trans, const vec2& vert) throw() {
    return vec2(  trans[0][0]*vert[0] + trans[0][1]*vert[1],
                  trans[1][0]*vert[0] + trans[1][1]*vert[1] );
  }
  
  // equivalent to this = trans * this, NOT this = this*trans
  static inline vec2& operator*=(vec2& v, const mat2& trans) throw() {
    v = trans * v;
    return v;
  }
  
  struct Segment {
    vec2 start;
    vec2 end;
    
    public:
    explicit Segment() throw() {}
    explicit Segment(const vec2& s, const vec2& e) throw() : start(s), end(e) {
      if( ! ( s < e ) ) {
        start = e;
        end = s;
      }
    }
    Segment(const Segment& other) throw() : start(other.start), end(other.end) {}
    
    vec2 direction() const throw() {
      return end - start;
    }
    
    vec2& operator[](unsigned int idx) throw(InvalidIndexException) {
      switch( idx ) {
        case 0:
          return start;
        case 1:
          return end;
        default:
          throw InvalidIndexException();
      }
    }
    
    const vec2& operator[](unsigned int idx) const throw(InvalidIndexException) {
      switch( idx ) {
        case 0:
          return start;
        case 1:
          return end;
        default:
          throw InvalidIndexException();
      }
    }
    
    // for std::less, doesn't actually mean anything
    bool operator<(const Segment& other) const throw() {
      if( start != other.start ) {
        return start < other.start;
      }
      else {
        return end < other.end;
      }
    }
    bool operator==(const Segment& other) const throw() {
      return start == other.start && end == other.end;
    }
  }; // class Segment
  
  class Ray {
    protected:
    vec2 start;
    vec2 direction;
    
    public:
    explicit Ray() throw() : start(0,0), direction(1,0) {}
    explicit Ray(const vec2& o, const vec2& m) throw() : start(o), direction(m) {
      direction.normalize();
    }
    explicit Ray(const vec2& o, float angle) throw() :
    start(o), direction(cosf(angle), sinf(angle)) {}
    Ray(const Ray& other) throw() : start(other.start), direction(other.direction) {}
    
    const vec2& origin() const throw() {
      return start;
    }
    void setOrigin(const vec2& other) throw() {
      start = other;
    }
    const vec2& dir() const throw() {
      return direction;
    }
    void setDir(const vec2& other) throw() {
      direction = other;
      direction.normalize();
    }
    float angle() const throw() {
      return atan2(direction.y, direction.x);
    }
    void setAngle(float angle) throw() {
      direction.x = cos(angle);
      direction.y = sin(angle);
    }
    // gets the point x units along the ray
    vec2 getPoint(float x) const throw() {
      return origin() + (x*dir());
    }
    
    Ray& transform(const mat2& trans) throw() {
      start = trans * start;
      direction = (trans * direction).normalize();
      return *this;
    }
    
    Ray& rotateAboutStart(const mat2& trans) throw() {
      direction *= trans;
      return *this;
    }
    
    Ray& operator+=(const vec2& shift) throw() {
      start += shift;
      return *this;
    }
    
    Ray& operator+=(const Ray& transform) throw() {
      start += direction.getRotationMatrix() * transform.start;
      direction *= transform.direction.getRotationMatrix();
      return *this;
    }
    
    // Takes the given vector in the coordinate system where
    // the origin of the ray is the origin and the ray points
    // along the positive x axis
    vec2 transformVecToAbsolute(const vec2& vec) const throw() {
      vec2 result = dir().getRotationMatrix() * vec;
      return (result += origin());
    }
    Ray transformRayToAbsolute(const Ray& ray) const throw() {
      Ray result;
      result.setOrigin(transformVecToAbsolute(ray.origin()));
      result.setDir(direction.getRotationMatrix()*ray.direction);
      return result;
    }

    // Takes the given vector in the absolute coordinate system
    // and transofrms it to a point in the coordinate system
    // with the origin at the start of the ray and pointin along it
    vec2 transformToLocal(const vec2& vec) const throw() {
      vec2 relative = vec - origin();
      return vec2(direction.dot(relative), direction.cross(relative));
    }
    
    Ray& operator=(const Ray& other) throw() {
      start = other.start;
      direction = other.direction;
      return *this;
    }
    
    // for std::less
    bool operator<(const Ray& other) const throw() {
      if( start != other.start ) {
        return start < other.start;
      }
      else {
        return direction < other.direction;
      }
    }
    bool operator==(const Ray& other) const throw() {
      return start == other.start && direction == other.direction;
    }
  }; // class Ray
  
  class Circle {
    public:
    vec2 center;
    protected:
    float _radius;
    
    public:
    explicit Circle(const vec2& c, float r) throw() : center(c), _radius(fabsf(r)) {}
        
    const float radius() const throw() {
      return _radius;
    }
    void setRadius(float rad) throw() {
      _radius = fabsf(rad);
    }
    
    bool contains(const vec2& p) {
      float dx = p.x - center.x;
      float dy = p.y - center.y;
      return dx*dx + dy*dy < _radius * _radius;
    }
  }; // class Circle
  
  // doesn't have to be a wall, but I need a name...
  // negative if there's no intersection
  float distance(const Ray& ray, const Segment& wall) throw();
  // Returns the shortest vector from that point to a segment
  vec2 pointToSeg(const vec2& c, const Segment& wall, float *param = NULL) throw();
  bool intersect(const Circle& c, const Segment& wall) throw();
} // namespace math

#endif // __GEOMETRY_H__
