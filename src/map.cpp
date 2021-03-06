/*
 *  map.cpp
 *  sim
 */
#include <iostream>
#include <fstream>
#include <climits>
#ifndef NO_GUI
#ifndef __APPLE__
#include <GL/gl.h>
#else
#include <OpenGL/gl.h>
#endif
#endif
#include "geometryio.h"
#include "map.h"
using namespace math;
using namespace sim;

Map* sim::read_map(const char* path) {
  std::ifstream input(path);
  if( !input ) {
    return NULL;
  }

  Map * result = new Map;
  
  std::string astring;
  for(input >> astring; input; input >> astring ) {
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
      input >> result->start;
    }
    else if( astring == "wall" ) {
      Wall w;
      input >> w;
      result->walls.push_back(w);
    }
    else if( astring == "door" ) {
      Door d;
      input >> d;
      result->doors.push_back(d);
    }
    else if( astring == "candle" ) {
      Candle c;
      input >> c;
      result->candles.push_back(c);
    }
    else {
      std::cerr << "Unrecognized token in map at " << path << ": " << astring << "\n";
      input.ignore(LONG_MAX, '\n');
    }
  }
  
  input.close();
  
  result->walls.push_back(Wall(vec2(0, 0), vec2(result->width, 0)));
  result->walls.push_back(Wall(vec2(result->width, 0),
                               vec2(result->width, result->height)));
  result->walls.push_back(Wall(vec2(result->width, result->height),
                               vec2(0, result->height)));
  result->walls.push_back(Wall(vec2(0, 0), vec2(0, result->height)));
  
  return result;
}

void sim::draw_map(Map * map, bool start, bool doCandles) {
#ifndef NO_GUI
  glMatrixMode(GL_MODELVIEW);
  glLineWidth(2.0);
  glBegin(GL_LINES);
  glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
  for( unsigned int i = 0; i < map->walls.size(); i++ ) {
    glVertex2f(map->walls[i][0][0],map->walls[i][0][1]);
    glVertex2f(map->walls[i][1][0],map->walls[i][1][1]);
  }
  glEnd();

  glLineWidth(1.0f);
  glBegin(GL_LINES);
  glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
  for( unsigned int i = 0; i < map->doors.size(); i++ ) {
    glVertex2f(map->doors[i][0][0],map->doors[i][0][1]);
    glVertex2f(map->doors[i][1][0],map->doors[i][1][1]);
  }
  glEnd();

  if( start ) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glPushMatrix();
    glTranslatef(map->start.origin().x, map->start.origin().y, 0.0);
    glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
    glRectd(-10, -10, 10, 10);
    glPopMatrix();
  }
  
  if( doCandles ) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
    const std::vector<Candle>& candles = map->candles;
    for( unsigned int i = 0; i < candles.size(); i++) {
      glPushMatrix();
      glTranslatef(candles[i][0], candles[i][1], 0.0);
      glRectd(-2, -2, 2, 2);
      glPopMatrix();
    }
  }
  glFlush();
#endif
}
