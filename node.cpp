#include <iostream>
#include "node.h"

Node:getXYZ(int c1, int c2, int c3){
  Node::x = c1;
  Node::y = c2;
  Node::z = c3;
}

Node:getXYZ(Node neighbor, int index){
  std::cout<<Node.neighbors[0]<<std::endl;
  Node::neighbors[index] = neighbor;
}

