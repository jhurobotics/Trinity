/*
 *  Nodel.h
 */

#include <vector>
#include "geometry.h"

#ifndef __NODEL_H__
#define __NODEL_H__



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

namespace math {

    class node{
      public:
        node* paths[4];
        Circle position;
        int checked;   // true when robot has passed through this node
        node* parent;
        int number; // node id for trouble shooting
        int searched;  // for traversing the graph
        node(int id=0) : position(vec2(0,0), 5){
           for(int i=0; i<4; ++i) {
             paths[i]=NULL;
           }
           checked=0;
           parent=0;
           number=id;
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
            return (position.center - next->position.center).mag();
        }
        void Connect(node* n, int dir) {
           paths[(dir)%4]=n;
           n->setnode((dir+2)%4,this);
        } 
        void add(int dir){
          Connect(new node(), dir);
          paths[(dir)%4]->parent=this;
        }
        Circle getPosition() {
          return position;
        }
        void setPosition(const Circle& newPos) {
          position = newPos;
        }
    };

    // represents the entire known maze and the bot inside it
    class graph {
      public:
        node* H;  // home node
        node* cur;  // current node
        int dir;  // current direction (N,S,E,W)
        std::vector<node*> route;   // route will be filled when traverse is called
        int nodecount;    // give each node a unique ID for trouble shooting
        graph(bool left, bool right, bool front, bool back=false) {
            H=new node(); 
            cur=H; 
            dir=0;
            route = std::vector<node*>();
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
        
        
        int traverse(node *n, int level, int searched, int maxdepth=0);
        
        // looks for closest unchecked nodes
        // returns a direction (-1,0,1,2)
        // returns a direction to turn in relative to the current direction
        int traverse();
        
        // returns false if there are no nodes left on the queue
        // traverse() needs to be called to fill the queue
        // bool nextDirection() {}
        
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
}



#endif // __NODEL_H__
