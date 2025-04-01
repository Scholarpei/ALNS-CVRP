// 节点类
#include "Node.h"
#include<iostream>
#include<iomanip>

using namespace std;

Node::Node(int id, double x, double y, int demand)
        : id(id), x(x), y(y), demand(demand) {}