#include <iostream>
#include <vector>
#include <cmath>

/*
// Minimal class to replace std::vector in Arduino
template<typename Data>
class Vector {
   size_t d_size; // Stores no. of actually stored objects
   size_t d_capacity; // Stores allocated capacity
   Data *d_data; // Stores data
   public:
     Vector() : d_size(0), d_capacity(0), d_data(0) {}; // Default constructor
     Vector(Vector const &other) : d_size(other.d_size), d_capacity(other.d_capacity), d_data(0) { d_data = (Data *)malloc(d_capacity*sizeof(Data)); memcpy(d_data, other.d_data, d_size*sizeof(Data)); }; // Copy constuctor
     ~Vector() { free(d_data); }; // Destructor
     Vector &operator=(Vector const &other) { free(d_data); d_size = other.d_size; d_capacity = other.d_capacity; d_data = (Data *)malloc(d_capacity*sizeof(Data)); memcpy(d_data, other.d_data, d_size*sizeof(Data)); return *this; }; // Needed for memory management
     void push_back(Data const &x) { if (d_capacity == d_size) resize(); d_data[d_size++] = x; }; // Adds new value. If needed, allocates more space
     size_t size() const { return d_size; }; // Size getter
     Data const &operator[](size_t idx) const { return d_data[idx]; }; // Const getter
     Data &operator[](size_t idx) { return d_data[idx]; }; // Changeable getter
   private:
     void resize() { d_capacity = d_capacity ? d_capacity*2 : 1; Data *newdata = (Data *)malloc(d_capacity*sizeof(Data)); memcpy(newdata, d_data, d_size * sizeof(Data)); free(d_data); d_data = newdata; };// Allocates double the old space
}; 


*/


#define LFIR 2  // left front
#define RFIR 3  // right front
#define LBIR 4  // left back
#define RBIR 5  // right back
#define FIR  6  // front
#define BIR  7  // back
#define LED 13
#define Left   1   // turn left
#define Right -1   // turn right
#define N 0
#define W 1
#define S 2
#define E 3


using namespace std; 

static int nodecount=1;

class node{
  public:
    node* paths[4];
    float position[2];   // (x,y) of the node relative to home
    int checked;   // true when robot has passed through this node
    node* parent;
    int number; // node id for trouble shooting
    int searched;  // for traversing the graph
    node() {
       for(int i=0; i<4; ++i) {
         paths[i]=NULL;
       }
       position[0]=0;
       position[1]=0;
       checked=0;
       parent=0;
       number=nodecount++;
       searched=false;
    }
    // 0=north; 1=west; 2=south; 3=east;
    void setnode(int dir, node* obj=0) {
      paths[dir]=obj;
    }
    node* getnode(int dir) {
      return paths[(dir)%4]; 
    }
    // returns the absolute direction of an adjacent node
    // returns false if the node is not connected to this
    bool getdir(node* node, int &dir) {
        for(int i=0; i<4; ++i) {
            if(paths[i]==node) {dir=i; return true;}
        }
        return false;
    }
    float getDistance(int dir) {
        node* next = getnode(dir);
        float x = position[0]-next->position[0];
        float y = position[1]-next->position[1];
        return sqrt(x*x+y*y);
    }
    void Connect(node* n, int dir) {
       paths[(dir)%4]=n;
       n->setnode((dir+2)%4,this);
    } 
    void add(int dir){
      Connect(new node(), dir);
      paths[(dir)%4]->parent=this;
    }
    void getPosition(float &x, float &y) {
        x=position[0];
        y=position[1];
    }
    void setPosition(float x, float y) {
        position[0]=x;
        position[1]=y;
    }
};

// represents the entire known maze and the bot inside it
class graph {
  public:
    node* H;  // home node
    node* cur;  // current node
    int dir;  // current direction (N,S,E,W)
    vector<node*> route;   // route will be filled when traverse is called
    graph(bool left, bool right, bool front, bool back=false) {
        H=new node(); 
        cur=H; 
        dir=0;
        route = vector<node*>();
        if(left) H->add(W);
        if(right) H->add(E);
        if(front) H->add(N);
        if(back) H->add(S);
        H->checked=true;
    }
    
    // adds nodes to the current node
    // TODO:  What happens if nodes already exist?  this means the robot got confused, but what should we do about it?
    void expand(bool left, bool right, bool front) {
     if(front){
       cur->add(dir);
     } 
     if(left){
       cur->add(dir+W);
     }      
     if(right){
       cur->add(dir+E);
     }       
    }
    
    
    void turn(int Direction) {
      dir+=Direction; 
      dir%=4;
    }
    
    // returns false if the robot is facing a wall
    bool move() {
        node *next = cur->getnode(dir);
        if(next) {
            cur=next;
            cur->checked=true;
            return true;
        }
        return false;
        
    }
    
    node * getNode(int Direction) {
        return cur->getnode((dir+Direction)%4);
    }
    
    
    int traverse(node *n, int level, int searched, int maxdepth=0) {
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
    int traverse() {
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
    
    // returns false if there are no nodes left on the queue
    // traverse() needs to be called to fill the queue
    bool nextDirection() {}
    
    // the robot physically does a depth first search of the maze
    // returns the next direction to travel in
    int quickTraverse(int default_dir=Right) {
        int direction=0;
        cur->getdir(cur->parent,direction);
        for(int i=1; i<4; ++i ) {
            node* next = cur->getnode(direction+i*default_dir);
            if(next && !next->checked) {return (direction+i*default_dir-dir)%4;}
        }
        return (direction-dir)%4;  // go back the way you came (assuming that is the parent node)
    }
    
};


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
         if(!p||p->searched^unsearched) continue;
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

#define NODE_TEST
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
