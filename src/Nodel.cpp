#include <iostream>
#include <vector>
#define boolean bool
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

int nodecount=1;

class node{
  private:
    node* paths[4];
  public:
    int checked;
    node* parent;
    int number; // trouble shooting
    boolean searched;  // troule shooting
    node() {paths[0]=0; paths[1]=0; paths[2]=0; paths[3]=0;checked=0;
       parent=0;
       number=nodecount++;
       searched=false;
    }
    // 0=north; 1=west; 2=south; 3=east;
    void setnode(int dir, node* obj=0) {
      paths[dir]=obj;
    }
    node* getnode(int dir) {
      return paths[dir]; 
    }
    void Connect(node* n, int dir) {
       paths[dir]=n;
       n->setnode((dir+2)%4,this);
    } 
    void add(int dir){
      Connect(new node(), dir);
      paths[dir]->parent=this;
    }
};

// represents the entire known maze and the bot inside it
class graph {
  private:
  
  public:
    node* H;  // home node
    node* cur;  // current node
    int dir;  // current direction (N,S,E,W)
    graph(boolean left, boolean right, boolean front, boolean back=false) {
        H=new node(); 
        cur=H; 
        dir=0;
        if(left) H->add(W);
        if(right) H->add(E);
        if(front) H->add(N);
        if(back) H->add(S);
        H->checked=true;
    }
    
    // adds information to a node next to the current node
    void update(boolean left, boolean right, boolean front) {
      node* next = cur->getnode(dir);
      next->checked=true;
     if(front){
       next->add(dir);
     } 
     if(left){
       next->add((dir+W)%4);
     }      
     if(right){
       next->add((dir+E)%4);
     }       
     cur=next;
    }
    
    void turn(int Direction) {
      dir+=Direction; 
      dir%=4;
    }
    boolean move() {
        if(cur->getnode(dir)) {cur=cur->getnode(dir); return true;}
        return false;
        
    }
    
    node * getNode(int Direction) {
        return cur->paths[(dir+Direction)%4];
    }
    
    /*
    // looks for closest unchecked nodes
    // returns a direction (-1,0,1,2)
    int traverse() {
      int parentdir=0;
      for(int i=0; i<4; ++i) {   // find direction of parent
        if(cur->getnode(i)==cur->parent) 
        {parentdir=i; break;}
        
      }
      int left = (parentdir-1)%4;
      int right= (parentdir+1)%4;
      int front= (parentdir+2)%4;
      if(cur->getnode(left)&&!cur->getnode(left)->checked) return -1;
      if(cur->getnode(right)&&!cur->getnode(right)->checked) return 1;
      if(cur->getnode(front)&&!cur->getnode(front)->checked) return 0;
      // if at dead end??
    }  */
    
    
    int traverse(node *n, int level, int maxdepth=0) {
        if(!n->checked) {return level;}  // return the distance to the unchecked node
        if(maxdepth>0 && level>=maxdepth) {return -1;} // there is no unchecked node here
        int maxdist = 0;
        for(int i=0; i<4; ++i) {
            if(n->paths[i]) {
                int dist = traverse(n->paths[i], level+1, maxdepth);
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
      for(int i=0; i<4; ++i) {
        int dist = traverse(getNode(i),1);
        if(dist>maxdist) {
            Direction = i-1;
            maxdist=dist;
        }
      }
      if(maxdist==0) {  // there are no more unchecked nodes
         // what do??
      }
      
      return Direction;
    }
    
};


// for trouble shooting
void outputGraph(node* n) {
     cout << n->number;
     if(n->checked) cout << " CHECKED";
     cout << "\n";
     if(n->getnode(N)) cout << "  N " << n->getnode(N)->number << "\n";
     if(n->getnode(W)) cout << "  W " << n->getnode(W)->number << "\n";
     if(n->getnode(S)) cout << "  S " << n->getnode(S)->number << "\n";
     if(n->getnode(E)) cout << "  E " << n->getnode(E)->number << "\n";
     cout << "\n";
     n->searched=true;
     for(int i=0; i<4; ++i) {
         node* p=n->getnode(i);
         if(!p||p->searched) continue;
         outputGraph(n->getnode(i));
     }
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
  g->update(1,0,1);
  g->turn(W);
  g->update(0,0,1);
  g->update(0,0,0);
  g->turn(2);
  g->move();
  g->move();
  
  
  outputGraph(g->H);
  cout << "current = " << g->cur->number << "\n";
  cout << "heading = " << directionString(g->dir) << "\n";
  cout << "go " << relativeDirString(g->traverse()) << "\n";
  //cout << "go " << g->traverse() << "\n";
  
  
  cout << "\n\nDONE";
  cin.get();    
}

#endif