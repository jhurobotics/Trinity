/*
 *  main.cpp
 *  sim
 */

#include "map.h"
#include <iostream>

int main( void )
{
  sim::Map * m = sim::read_map("basic_map");
  std::cout << "map has " << m->walls.size() << " walls\n";
  std::cout << "map has " << m->doors.size() << " doors\n";
  std::cout << "map has " << m->candles.size() << " candles\n";
  delete m;
  return 0;
}
