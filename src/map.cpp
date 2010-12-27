/*
 *  map.cpp
 *  sim
 */
#include <iostream>
#include <fstream>
#include <OpenGL/gl.h>

#include "map.h"
using namespace sim;

Map* sim::read_map(const char* path) {
  std::ifstream input(path);
  
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

void sim::draw_map(Map * map, bool start, bool doCandles) {
  glMatrixMode(GL_MODELVIEW);
  glLineWidth(2.0);
  glBegin(GL_LINES);
  glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
  for( int i = 0; i < map->walls.size(); i++ ) {
    glVertex2f(map->walls[i][0][0],map->walls[i][0][1]);
    glVertex2f(map->walls[i][1][0],map->walls[i][1][1]);
  }
  glEnd();

  glLineWidth(1.0f);
  glBegin(GL_LINES);
  glColor3f(0.0f, 0.0f, 1.0f);
  for( int i = 0; i < map->doors.size(); i++ ) {
    glVertex2f(map->doors[i][0][0],map->doors[i][0][1]);
    glVertex2f(map->doors[i][1][0],map->doors[i][1][1]);
  }
  glEnd();

  if( start ) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glPushMatrix();
    glTranslatef(map->start[0], map->start[1], 0.0);
    glColor3f(0.0, 1.0, 0.0);
    glRectd(-10, -10, 10, 10);
    glPopMatrix();
  }
  
  if( doCandles ) {
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glColor3f(1.0, 0.0, 0.0);
    const std::vector<Candle>& candles = map->candles;
    for( int i = 0; i < candles.size(); i++) {
      glPushMatrix();
      glTranslatef(candles[i][0], candles[i][1], 0.0);
      glRectd(-2, -2, 2, 2);
      glPopMatrix();
    }
  }
  glFlush();
}