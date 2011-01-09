/*
 *  geometry.h
 *  sim
 */

#ifndef __GEOMETRY_H__
#define __GEOMETRY_H__

#include <cmath>
#include <cstring>

namespace math {
  
  class InvalidIndexException {};
  
  class mat2 {
    public:
    // [row][col]
    float data[2][2];
    
    mat2() {
      data[0][0] = data[1][1] = 1;
      data[0][1] = data[1][0] = 0;
    }
    mat2(float d11, float d12, float d21, float d22) {
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
    
    float det() const {
      return data[0][0] * data[1][1] - data[0][1] * data[1][0];
    }
    
    mat2 operator*(const mat2& other) {
      mat2 result;
      result.data[0][0] = data[0][0]*other.data[0][0] + data[0][1]*other.data[1][0];
      result.data[0][1] = data[0][0]*other.data[0][1] + data[0][1]*other.data[1][1];
      result.data[1][0] = data[1][0]*other.data[0][0] + data[1][1]*other.data[1][0];
      result.data[1][1] = data[1][0]*other.data[0][1] + data[1][1]*other.data[1][1];
      return result;
    }
  }; // class mat2
  
  mat2 getRotationMatrix(float angle);
  
  class mat3 {
  public:
    // [row][col]
    float data[3][3];
    
    mat3() {
      memset(data, 0, 9*sizeof(float));
      data[0][0] = data[1][1] = data[2][2];
    }
        
    const float* operator[](unsigned int idx) const throw(InvalidIndexException) {
      switch(idx) {
        case 0:
          return data[0];
        case 1:
          return data[1];
        case 2:
          return data[2];
        default:
          throw InvalidIndexException();
      }
    }
    
    mat2 minorExclude(unsigned int row, unsigned int col) const {
      mat2 result;
      unsigned int inRow = 0;
      unsigned int inCol = 0;
      for(unsigned int r = 0; r < 3; r++) {
        if( r == row ) {
          continue;
        }
        for( unsigned int c = 0; c < 3; c++ ) {
          if( c == col ) {
            continue;
          }
          result.data[inRow][inCol] = data[r][c];
          inCol++;
        }
        inRow++;
      }
      return result;
    }
    
    float det() const {
      return data[0][0] * minorExclude(0, 0).det()
           - data[0][1] * minorExclude(0, 1).det()
           + data[0][2] * minorExclude(0, 2).det();
    }
  }; // class mat3
  
  class vec2 {
    public:
    float x;
    float y;
    
    explicit vec2() : x(0), y(0) {}
    explicit vec2(float nx, float ny) : x(nx), y(ny) {}
    vec2(const vec2& other) : x(other.x), y(other.y) {}
    
    float& operator[](int idx) {
      switch(idx) {
        case 0:
          return x;
        case 1:
          return y;
        default:
          throw;
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
    
    float mag_sq() const {
      return x * x + y * y;
    }
    
    float mag() const {
      return sqrt(mag_sq());
    }
    
    vec2 operator+(const vec2& other) const {
      return vec2( x + other.x, y + other.y );
    }
    vec2 operator-(const vec2& other) const {
      return vec2( x - other.x, y - other.y );
    }
    float dot(const vec2& other) const {
      return x * other.x + y * other.y;
    }
    vec2 operator*(float scale) const {
      return vec2( x * scale, y * scale );
    }
    vec2 operator/(float scale) const {
      return vec2( x / scale, y / scale );
    }
    vec2& operator=(const vec2& other) {
      x = other.x;
      y = other.y;
      return *this;
    }
    vec2& operator+=(const vec2& other) {
      x += other.x;
      y += other.y;
      return *this;
    }
    vec2& operator-=(const vec2& other) {
      x -= other.x;
      y -= other.y;
      return *this;
    }
    vec2& operator*=(float scale) {
      x *= scale;
      y *= scale;
      return *this;
    }
    vec2& operator/=(float scale) {
      x /= scale;
      y /= scale;
      return *this;
    }
    float& operator[](unsigned int idx) {
      switch(idx) {
        case 0:
          return x;
        case 1:
          return y;
        default:
          throw;
      }
    }
    vec2& normalize() {
      return (*this) /= mag();
    }
    float cross(const vec2& other) const {
      return x*other.y - y*other.x;
    }
    
    vec2 transpose() {
      return vec2(y, x);
    }
        
    bool operator==(const vec2& other) const {
      return x == other.x && y == other.y;
    }
    bool operator!=(const vec2& other) const {
      return x != other.x || y != other.y;
    }
    
    // Assuming this vector is normalized
    // Get the rotation associated with this forward vector
    mat2 getRotationMatrix() const {
      return mat2( x, -y, y, x);
    }
    
    // for std::less, doesn't actually mean anything
    bool operator<(const vec2 other) const {
      if( x != other.x ) {
        return x < other.x;
      }
      else {
        return y < other.y;
      }
    }
    
  }; // class vec2
  
  static inline vec2 operator-(const vec2& arg) {
    return vec2(-(arg[0]), -(arg[1]));
  }
  static inline vec2 operator*(float scalar, const vec2& vector) {
    return vec2(vector[0] * scalar, vector[1] * scalar);
  }
  static inline vec2 operator/(float scalar, const vec2& vector) {
    return vec2(scalar / (vector[0]), scalar / (vector[1]) );
  }
  static inline vec2 operator*(const mat2& trans, const vec2& vert) {
    return vec2(  trans[0][0]*vert[0] + trans[0][1]*vert[1],
                  trans[1][0]*vert[0] + trans[1][1]*vert[1] );
  }
  
  // equivalent to this = trans * this, NOT this = this*trans
  static inline vec2& operator*=(vec2& v, const mat2& trans) {
    v = trans * v;
    return v;
  }
  
  struct Segment {
    vec2 start;
    vec2 end;
    
    public:
    explicit Segment() {}
    explicit Segment(const vec2& s, const vec2& e) : start(s), end(e) {
      if( ! ( s < e ) ) {
        start = e;
        end = s;
      }
    }
    Segment(const Segment& other) : start(other.start), end(other.end) {}
    
    vec2 direction() const {
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
    bool operator<(const Segment& other) const {
      if( start != other.start ) {
        return start < other.start;
      }
      else {
        return end < other.end;
      }
    }
    bool operator==(const Segment& other) const {
      return start == other.start && end == other.end;
    }
  }; // class Segment
  
  class Ray {
    protected:
    vec2 start;
    vec2 direction;
    
    public:
    Ray() : start(0,0), direction(1,0) {}
    Ray(const vec2& o, const vec2& m) : start(o), direction(m) {
      direction.normalize();
    }
    Ray(const Ray& other) : start(other.start), direction(other.direction) {}
    
    const vec2& origin() const {
      return start;
    }
    void setOrigin(const vec2& other) {
      start = other;
    }
    const vec2& dir() const {
      return direction;
    }
    void setDir(const vec2& other) {
      direction = other;
      direction.normalize();
    }
    // gets the point x units along the ray
    vec2 getPoint(float x) const {
      return origin() + (x*dir());
    }
    
    Ray& transform(const mat2& trans) {
      start = trans * start;
      direction = (trans * direction).normalize();
      return *this;
    }
    
    Ray& rotateAboutStart(const mat2& trans) {
      direction *= trans;
      return *this;
    }
    
    Ray& operator+=(const vec2& shift) {
      start += shift;
      return *this;
    }
    
    Ray& operator+=(const Ray& transform) {
      start += direction.getRotationMatrix() * transform.start;
      direction *= transform.direction.getRotationMatrix();
      return *this;
    }
    
    // Takes the given vector in the coordinate system where
    // the origin of the ray is the origin and the ray points
    // along the positive x axis
    vec2 transformToAbsolute(const vec2& vec) const {
      vec2 result = dir().getRotationMatrix() * vec;
      return (result += origin());
    }

    // Takes the given vector in the absolute coordinate system
    // and transofrms it to a point in the coordinate system
    // with the origin at the start of the ray and pointin along it
    vec2 transformToLocal(const vec2& vec) const throw() {
      vec2 relative = vec - origin();
      float t = dir().dot(relative);
      float s = dir().cross(relative);
      return vec2(t, s);
    }
    
    Ray& operator=(const Ray& other) {
      start = other.start;
      direction = other.direction;
      return *this;
    }
    
    // for std::less
    bool operator<(const Ray& other) const {
      if( start != other.start ) {
        return start < other.start;
      }
      else {
        return direction < other.direction;
      }
    }
    bool operator==(const Ray& other) const {
      return start == other.start && direction == other.direction;
    }
  }; // class Ray
  
  class Circle {
    protected:
    vec2 _center;
    float _radius;
    
    public:
    Circle(const vec2& c, float r) : _center(c), _radius(fabsf(r)) {}
    
    const vec2& center() const {
      return _center;
    }
    void setCenter(const vec2& other) {
      _center = other;
    }
    
    const float radius() const {
      return _radius;
    }
    void setRadius(float rad) {
      _radius = fabsf(rad);
    }
  }; // class Circle
  
  // doesn't have to be a wall, but I need a name...
  // negative if there's no intersection
  float distance(const Ray& ray, const Segment& wall);
  // Returns the shortest vector from that point to a segment
  inline vec2 pointToSeg(const vec2& c, const Segment& wall, float *param = NULL);
  bool intersect(const Circle& c, const Segment& wall);
} // namespace math

#endif // __GEOMETRY_H__
