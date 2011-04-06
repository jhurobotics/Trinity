/*
 *  Graph.cpp
 *  sim
 */
#include <list>
#include <map>
#include <queue>
#include <algorithm>
#include <limits.h>
#include "graph.h"
using namespace math;
using namespace std;
using namespace robot;

const float Node::RESOLUTION = 3.0;

Graph::~Graph() {
  set<Node*>::iterator nEnd = vertices.end();
  for( set<Node*>::iterator iter = vertices.begin(); iter != nEnd; iter ++ ) {
    delete *iter;
  }
}

Node * Graph::onNode(const vec2& pos ) {
  set<Node*>::iterator end = vertices.end();
  for( set<Node*>::iterator iter = vertices.begin(); iter != end; iter++ ) {
    if( (*iter)->position.contains(pos) ) {
      return *iter;
    }
  }
  return NULL;
}

struct D_Data {
  Node * n;
  D_Data * prev;
  float dist;
  D_Data(Node * _n, D_Data * _p, float d = -1.0f)
    : n(_n), prev(_p), dist(d) {}
  bool operator<(const D_Data& o) const {
    return dist < o.dist;
  }
};

/*class D_compare {
  public:
  bool operator()(const D_Data * a, const D_Data * b) {
    return a->dist < b->dist;
  }
};*/

bool D_compare(const D_Data * a, const D_Data * b) {
  return a->dist < b->dist;
}


Node * Graph::getObjective(const vec2& pos) throw(EmptyGraphException, AllCheckedException) {
  if( currentObjective ) {
    if( !currentObjective->position.contains(pos) ) {
      return currentObjective;
    }
    else {
      route.pop();
      if( !route.empty() ) {
        currentObjective = route.top();
        return currentObjective;
      }
    }
  }
  
  if( vertices.empty() ) {
    throw EmptyGraphException();
  }
  
  // do dijkstra's
  Node * start = NULL;
  if( !currentObjective ) {
    start = onNode(pos);
    // what if no objective and not on a node? ... does that need implementing?
  }
  else {
    start = currentObjective;
  }
  D_Data * startData = new D_Data(start, NULL, 0);
  deque<D_Data*> unexplored;
  unexplored.push_front(startData);
  
  map<Node*, D_Data*> enqueued;
  enqueued[start] = startData;
  D_Data * closest = NULL;
  // do dijkstra's to find shortest path to unchecked nodes, add those to destinations
  for( unsigned int finCount = 0; finCount < vertices.size() && !unexplored.empty() ; finCount++) {
    D_Data * data = unexplored.front();
    pop_heap(unexplored.begin(), unexplored.end(), D_compare);
    const Node * node = data->n;
    
    typedef set<Node*>::const_iterator neighbor_iter_t;
    // iterate over the links
    neighbor_iter_t linkEnd = node->links.end();
    for( neighbor_iter_t linkIter = node->links.begin(); linkIter != linkEnd; linkIter ++ ) {
      Node * neigh = (*linkIter);
      // get the data on this node
      D_Data * neighData = enqueued[neigh];
      if( !neighData ) {
        // enqueue this node if it hasn't been already
        neighData = new D_Data(neigh, data);
        unexplored.push_back(neighData);
        enqueued[neigh] = neighData;
      }
      // see if this is a shorter path
      float dist = (node->position.center - neigh->position.center).mag() + data->dist;
      if( neighData->dist < 0 || dist < neighData->dist ) {
        neighData->dist = dist;
        neighData->prev = data;
      }      
    }
    // finish this node
    if( !node->checked ) {
      if( !closest || data->dist < closest->dist ) {
        closest = data;
      }
    }
    // fix the heap, cuz we modified the elements behind it's back
    make_heap(unexplored.begin(), unexplored.end(), D_compare);
  }
  
  // store the route
  if( !closest ) {
    throw AllCheckedException();
  }
  
  while( closest != startData ) {
    route.push(closest->n);
    closest = closest->prev;
  }
  currentObjective = route.top();
  
  // delete all the metadata
  map<Node*, D_Data*>::iterator end = enqueued.end();
  for( map<Node*, D_Data*>::iterator iter = enqueued.begin(); iter != end; iter++ ) {
    delete iter->second;
  }
  
  return currentObjective;
}

#include <iostream>
#include <fstream>
#include <string>
#include "geometryio.h"

static void read_bool(istream& input, bool * dest) {
  string astring;
  input >> astring;
  if( astring == "true" ) {
    (*dest) = true;
  }
  else {
    (*dest) = false;
  }
}

void robot::read_graph(Graph * g, const char * path) {
  ifstream input(path);
  
  string astring;
  Node * curNode = NULL;
  while( input ) {
    input >> astring;
    if( astring[0] == '#' ) { // comment, skip line
      input.ignore(LONG_MAX, '\n');
    }
    else if( astring == "NODE" ) {
      input >> astring;
      curNode = new Node();
      g->vertices.insert(curNode);
      g->nodeByName[astring] = curNode;
    }
    else if( astring == "room" ) {
      if( !curNode ) {
        cout << "no current node for room property\n";
        break;
      }
      read_bool(input, &(curNode->room));
    }
    else if( astring == "checked" ) {
      if( !curNode ) {
        cout << "no current node for checked property\n";
        break;
      }
      read_bool(input, &(curNode->checked));
    }
    else if( astring == "pos" ) {
      if( !curNode ) {
        cout << "no current node for pos property\n";
        break;
      }
      input >> curNode->position.center;
    }
    else if( astring == "EDGE" ) {
      string bstring;
      input >> astring >> bstring;
      Node * aNode = g->nodeByName[astring];
      Node * bNode = g->nodeByName[bstring];
      aNode->links.insert(bNode);
      bNode->links.insert(aNode);
    }
  }
}

#ifndef __APPLE__
#include <GL/gl.h>
#else
#include <OpenGL/gl.h>
#endif

#define RED 1.0, 0.0, 0.0
#define GREEN 0.0, 1.0, 0.0
#define BLUE 0.0, 0.0, 1.0
#define WHITE 1.0, 1.0, 1.0

void Graph::draw() {
  set<Node*>::iterator vEnd = vertices.end();
  for( set<Node*>::iterator iter = vertices.begin(); iter != vEnd; iter++ ) {
    Node * n = *iter;
    // Draw links to neighbors
    neighbor_iter_t end = n->links.end();
    glColor4f(GREEN, 1.0);
    glBegin(GL_LINES);
    for( neighbor_iter_t iter = n->links.begin(); iter != end; iter++ ) {
      glVertex2f(n->position.center.x, n->position.center.y);
      glVertex2f((*iter)->position.center.x, (*iter)->position.center.y);
    }
    glEnd();
    
    glPushMatrix();
    glTranslatef( n->position.center.x, n->position.center.y, 0);
    
    if( n->room ) {
      glColor4f(RED, 1.0);
    }
    else {
      glColor4f(GREEN, 1.0);
    }
    
    if( (*iter)->checked ) {
      glBegin(GL_TRIANGLE_FAN);
      glVertex2f( 0, 0);
    }
    else {
      glBegin(GL_LINE_STRIP);
    }
    
    int angleCount = 32;
    for(int i = 0; i < angleCount+1; i++) {
      glVertex2f(4*cos(((float)i)/((float)angleCount)*2*M_PI), 4*sin(((float)i)/((float)angleCount)*2*M_PI));
    }
    
    glEnd();
    glPopMatrix();
  }
  
  /*glColor4f(GREEN, 1.0);
  set<Edge*>::iterator eEnd = edges.end();
  for( set<Edge*>::iterator iter = edges.begin(); iter != eEnd; iter++ ) {
    Edge * e = *iter;
    glBegin(GL_LINES);
    glVertex2f(e->start->position.center.x, e->start->position.center.y);
    glVertex2f(e->end->position.center.x, e->end->position.center.y);
    glEnd();
  }*/
}
