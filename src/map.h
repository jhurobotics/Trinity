/*
 *  map.h
 *  sim
 */

#ifndef __MAP_H__
#define __MAP_H__

#include <vector>
#include <string>
#include <cmath>

namespace sim {
  
  class vec2 {
    public:
    
    float x;
    float y;
    
    explicit vec2() : x(0), y(0) {}
    explicit vec2(float nx, float ny) : x(nx), y(ny) {}
    
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
    
    float operator[](int idx) const {
      switch(idx) {
        case 0:
          return x;
        case 1:
          return y;
        default:
          throw;
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
    
    bool operator==(const vec2& other) const {
      return x == other.x && y == other.y;
    }
    bool operator!=(const vec2& other) const {
      return x != other.x || y != other.y;
    }
    
    // for std::less
    bool operator<(const vec2 other) const {
      if( x != other.x ) {
        return x < other.x;
      }
      else {
        return y < other.y;
      }
    }
    
  };
  
  static inline vec2 operator-(const vec2& arg) {
    return vec2(-(arg[0]), -(arg[1]));
  }
  static inline vec2 operator*(float scalar, const vec2& vector) {
    return vec2(vector[0] * scalar, vector[1] * scalar);
  }
  static inline vec2 operator/(float scalar, const vec2& vector) {
    return vec2(scalar / (vector[0]), scalar / (vector[1]) );
  }
  
  typedef vec2 Candle;
  
  struct Segment {
    vec2 start;
    vec2 end;
    
    public:
    explicit Segment() {}
    explicit Segment(const vec2& s, const vec2& e) : start(), end() {
      if( s < e ) {
        start = s;
        end = e;
      }
      else {
        start = e;
        end = s;
      }
    }
    Segment(const Segment& other) : start(other.start), end(other.end) {}
    
    vec2 displacement() {
      return end - start;
    }
    
    vec2& operator[](unsigned int idx) {
      switch( idx ) {
        case 0:
          return start;
        case 1:
          return end;
        default:
          throw;
      }
    }
    
    // for std::less
    bool operator<(const Segment& other) const {
      if( start != other.start ) {
        return start < other.start;
      }
      else {
        return end < other.end;
      }
    }
  };
  typedef Segment Wall;
  typedef Segment Door;
  
  struct Map {
    int width;
    int height;
    vec2 start;
    std::vector<Wall> walls;
    std::vector<Door> doors;
    std::vector<Candle> candles;
  };
  extern "C" Map* read_map(const char * path);
  extern "C" void draw_map(Map * map, bool start, bool emptyCandles);
}
    
#endif // __MAP_H__
