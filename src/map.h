/*
 *  map.h
 *  sim
 */

#ifndef __MAP_H__
#define __MAP_H__

#include <vector>
#include "geometry.h"

namespace sim {
    
  typedef math::vec2 Candle;
  
  typedef math::Segment Wall;
  typedef math::Segment Door;
  
  struct Map {
    int width;
    int height;
    math::vec2 start;
    std::vector<Wall> walls;
    std::vector<Door> doors;
    std::vector<Candle> candles;
  }; // class Map
  
  Map* read_map(const char * path);
  void draw_map(Map * map, bool start, bool emptyCandles);
} // namespace sim
    
#endif // __MAP_H__
