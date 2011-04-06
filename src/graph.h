/*
 *  graph.h
 *  sim
 */

#include <set>
#include <stack>
#include <string>
#include "geometry.h"

#ifndef __GRAPH_H__
#define __GRAPH_H__

namespace robot {
  
  class Edge;
  
  class Node {
  public:
    static const float RESOLUTION;
    bool room;
    bool checked;
    math::Circle position;
    std::set<Node*> links;
    std::string name;
    
    Node() : room(false), checked(false), position( math::vec2(), RESOLUTION ), links() {}
    Node(const math::vec2& pos, bool r = false)
    : room(r), checked(false), position(pos, RESOLUTION), links()
    {}
  };
  typedef std::set<Node*>::iterator neighbor_iter_t;
    
  class Graph {
  public:
    
    class EmptyGraphException : public std::exception {
      virtual const char * what() const throw() {
        return "Attempted operation on an empty graph.";
      }
    };
    
    class AllCheckedException : public std::exception {
      virtual const char * what() const throw() {
        return "Searched for objective, but all nodes are already checked.";
      }
    };
    
    std::set<Node*> vertices;
    std::set<Edge*> edges;
    Node * currentObjective;
    std::stack<Node*> route;
    std::map<std::string, Node*> nodeByName;
    
    Graph() throw() : vertices(), edges(), currentObjective(NULL) {}
    ~Graph();
    
    // NULL if not on a Node
    Node * onNode(const math::vec2& pos);
    
    Node* getObjective(const math::vec2& pos) throw(EmptyGraphException, AllCheckedException);

    void draw();
  };
  
  void read_graph(Graph * g, const char * path);

} // namespace robot
#endif // __GRAPH_H__