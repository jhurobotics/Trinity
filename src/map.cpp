/*
 *  map.cpp
 *  sim
 */
#include <iostream>
#include <fstream>
#include <limits.h>
#include "map.h"
using namespace sim;

Map* sim::read_map(const std::string& path) {
  std::ifstream input(path.c_str());
  
  Map * result = new Map;
  
  std::string astring;
  while( input ) {
    input >> astring;
    if( astring[0] == '#' ) { // comment, skip line
      input.ignore(LONG_MAX, '\n');
    }
    else if( astring == "width" ) {
      input >> result->width;
    }
    else if( astring == "height" ) {
      input >> result->height;
    }
    else if( astring == "start" ) {
      input >> result->start[0] >> result->start[1];
    }
    else if( astring == "wall" ) {
      Wall w;
      input >> w[0][0] >> w[0][1] >> w[1][0] >> w[1][1];
      result->walls.push_back(w);
    }
    else if( astring == "door" ) {
      Door w;
      input >> w[0][0] >> w[0][1] >> w[1][0] >> w[1][1];
      result->doors.push_back(w);
    }
    else if( astring == "candle" ) {
      Candle c;
      input >> c[0] >> c[1];
      result->candles.push_back(c);
    }
  }
  
  result->walls.push_back(Wall(vec2(0, 0), vec2(result->width, 0)));
  result->walls.push_back(Wall(vec2(result->width, 0),
                               vec2(result->width, result->height)));
  result->walls.push_back(Wall(vec2(result->width, result->height),
                               vec2(0, result->height)));
  result->walls.push_back(Wall(vec2(0, 0), vec2(0, result->height)));
  
  return result;
}
