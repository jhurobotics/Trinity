/*
 *  map.h
 *  sim
 */

#include <vector>
#include "geometry.h"

#ifndef __MAP_H__
#define __MAP_H__

namespace sim {
    
  typedef math::vec2 Candle;
  
  typedef math::Segment Wall;
  typedef math::Segment Door;
  
  struct Map {
    int width;
    int height;
    math::Ray start;
    std::vector<Wall> walls;
    std::vector<Door> doors;
    std::vector<Candle> candles;
    
    Map& operator=(const Map& other) {
      width = other.width;
      height = other.height;
      start = other.start;
      walls = other.walls;
      doors = other.doors;
      candles = other.candles;
      return *this;
    }
  }; // class Map
  
  Map* read_map(const char * path);
  void draw_map(Map * map, bool start, bool emptyCandles);
} // namespace sim
    
#endif // __MAP_H__
