#include <iostream>
#include "Nodel.h"


using namespace std;


static int nodecount=1;
using namespace math;
int graph::traverse(node *n, int level, int searched, int maxdepth) {
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
int graph::traverse() {
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


// for trouble shooting
void outputGraph(node* n, bool unsearched) {
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
         node* p=n->getnode(i);
         if( !p || p->searched ^ unsearched ) continue;
         outputGraph(n->getnode(i),unsearched);
     }
}
void outputGraph(node* n) {
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

#ifdef NODE_TEST

int main()
{
  graph* g = new graph(0,0,1,0);
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
