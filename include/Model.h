#ifndef MODEL_H
#define MODEL_H

#include "Node.h"
//#include "Solution.h"
#include <vector>
#include <unordered_map>
#include <string>

class Model
{
public:
    int vehicleCount;        // 车数量
    int capacity;            // 车容量
    std::vector<Node> nodes; // 节点(客户)
    std::unordered_map<int, std::unordered_map<int, double>> distanceMatrix;

    //Solution bestSolution;

    Model(int vehicleCount, int capacity);

    void computeDistances();                            // 算距离
    void initialSolution();                             // 初始解
    //double evaluateSolution(const Solution& sol) const; // evaluate solution

    static Model loadFromFile(const std::string& filename); // load data

    void print() const;  // 新增打印方法
};

#endif
