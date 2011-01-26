/*
 *  Graph.h
 *  sim
 */

#ifndef __GRAPH_H__
#define __GRAPH_H__

#include <set>
#include <stack>
#include "geometry.h"

namespace robot {

  class Node {
    public:
    static const float RESOLUTION = 3.0;
    bool room;
    math::Circle loc;
    
    Node(const Node& other)
      : room(other.room), loc(other.loc)
    {}
    Node(const math::vec2& pos, bool r)
      : room(r), loc(pos, RESOLUTION)
    {}
  }; // class Node

  class Graph {
    Node middle;
    
    public:
    Graph() throw() : middle(math::vec2(124, 124), false) {}
    
    Node* getObjective(const math::vec2& pos) {
      return &middle;
    }
  }; // class Graph

} // namespace robot

#endif // __GRAPH_H__