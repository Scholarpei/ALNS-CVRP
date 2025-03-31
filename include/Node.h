#ifndef NODE_H
#define NODE_H

class Node {
public:
    int id;
    double x;
    double y;
    int demand;

    Node(int id = 0, double x = 0.0, double y = 0.0, int demand = 0)
        : id(id), x(x), y(y), demand(demand) {}
};

#endif