#include <iostream>
#include "Nodel.h"


using namespace std;


static int nodecount=1;
using namespace math;
int Graph::traverse(Node *n, int level, int searched, int maxdepth) {
    if(!n) {return 0;}
    if(n->searched==searched)  {  // this node was already traversed
        if(!n->checked) {return level;}
        return 0;
    }
    // cout << "traverse:  checking node " << n->number << endl;
    n->searched++;
    // if(!n->checked) {return level;}  // return the distance to the unchecked node
    // if(maxdepth>0 && level>=maxdepth) {return 0;} // there is no unchecked node here
    int maxdist = 0;
    if(!n->checked) {maxdist = level;}
    for(int i=0; i<4; ++i) {
        if(n->paths[i]) {
            int dist = traverse(n->paths[i], level+1, n->searched, maxdepth);
            maxdist = (dist>maxdist)?dist:maxdist;   // take maximum
            if(maxdist>maxdepth) {maxdepth=maxdist;}
        }
    }
    return maxdist;
}
    
    // looks for closest unchecked nodes
    // returns a direction (-1,0,1,2)
    // returns a direction to turn in relative to the current direction
int Graph::traverse() {
  int maxdist = 0;
  int Direction = 0;
  // vector<node*> searched = vector<node*>;
  cur->searched++;
  for(int i=0; i<4; ++i) {
    int dist = traverse(getNode(i),1,cur->searched);
    if(dist>maxdist) {
        Direction = i;
        maxdist=dist;
    }
  }
  if(maxdist==0) {  // there are no more unchecked nodes
     // what do??
  }
  
  return Direction;
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
    
    glColor4f(GREEN, 1.0);
    glBegin(GL_LINES);
    for( int i = 0; i < 4; i++ ) {
      Node * end = n->paths[i];
      if( end ) {
        glVertex2f(n->position.center.x, n->position.center.y);
        glVertex2f(end->position.center.x, end->position.center.y);
      }
    }
    glEnd();
  }
}

// for trouble shooting
void outputGraph(Node* n, bool unsearched) {
     cout << n->number;
     if(n->checked) cout << " CHECKED";
     cout << "\n";
     if(n->getnode(N)) cout << "  N " << n->getnode(N)->number << "\n";
     if(n->getnode(W)) cout << "  W " << n->getnode(W)->number << "\n";
     if(n->getnode(S)) cout << "  S " << n->getnode(S)->number << "\n";
     if(n->getnode(E)) cout << "  E " << n->getnode(E)->number << "\n";
     cout << "\n";
     n->searched=!unsearched;
     for(int i=0; i<4; ++i) {
         Node* p=n->getnode(i);
         if( !p || p->searched ^ unsearched ) continue;
         outputGraph(n->getnode(i),unsearched);
     }
}
void outputGraph(Node* n) {
    outputGraph(n,n->searched);
}

char directionString(int dir) {
     dir%=4;
     if(dir==N) return 'N';
     if(dir==S) return 'S';
     if(dir==E) return 'E';
     if(dir==W) return 'W';
     return '!';
}

// front, left, right, back
char relativeDirString(int dir) {
     dir=dir%4;
     if(dir==N) return 'F';
     if(dir==S) return 'B';
     if(dir==E) return 'R';
     if(dir==W) return 'L';
     return '!';
}

#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include "geometryio.h"
using namespace std;

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

void math::read_graph(Graph * g, const char * path) {
  static const vec2 NORTH( 0, 1);
  static const vec2 EAST ( 1, 0);
  ifstream input(path);
  
  string astring;
  std::map<string, Node*> nodes;
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
      nodes[astring] = curNode;
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
    else if( astring == "start" ) {
      g->H = curNode;
    }
    else if( astring == "EDGE" ) {
      string bstring;
      input >> astring >> bstring;
      Node * aNode = nodes[astring];
      Node * bNode = nodes[bstring];
      vec2 disp = aNode->position.center - bNode->position.center;
      float d;
      if( abs(d = disp.dot(NORTH)/disp.mag()) > 0.9 ) {
        if( d > 0 ) {
          bNode->paths[N] = aNode;
          aNode->paths[S] = bNode;
        }
        else {
          bNode->paths[S] = aNode;
          aNode->paths[W] = bNode;
        }
      }
      else {
        if( disp.dot(EAST) > 0 ) {
          bNode->paths[E] = aNode;
          aNode->paths[W] = bNode;
        }
        else {
          bNode->paths[W] = aNode;
          aNode->paths[E] = bNode;
        }
      }
      //g->edges.insert(new Edge(nodes[astring], nodes[bstring]));
    }
  }
}


#ifdef NODE_TEST

int main()
{
  Graph* g = new Graph(0,0,1,0);
  g->move();
  g->expand(1,0,1);
  g->turn(W);
  g->move();
  g->expand(0,0,1);
  g->move();
  g->expand(0,0,0);
  
  
  outputGraph(g->H);
  cout << "current = " << g->cur->number << "\n";
  cout << "heading = " << directionString(g->dir) << "\n" << "\n";
  //int dir = g->traverse();
  //cout << "go " << relativeDirString(dir) << "("<<directionString(dir+g->dir)<<")" << "\n";
  //cout << "go " << g->traverse() << "\n";
  
  // traverse
  //for(int i=0; i<3; ++i) {
  while(1) {
    int dir = g->quickTraverse();  // will fail if theres a loop
    // int dir = g->traverse();
    cout << "@" << g->cur->number << endl;
    cout << "go " << relativeDirString(g->traverse()) << "("<<directionString(dir+g->dir)<<")" << "\n";
    g->turn(dir);
    if(!g->getNode(0)->checked) {g->move(); break;}
    g->move();
  }
  
  cout << "@" << g->cur->number << endl;
  
  //outputGraph(g->H);
  
  
  //cout << "\n\nDONE";
  //cin.get();    
}

#endif
