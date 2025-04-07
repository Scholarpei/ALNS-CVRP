#ifndef MODEL_H
#define MODEL_H

#include "Node.h"
#include <vector>
#include <unordered_map>
#include <memory>
// #include "Solution.h"
#include <string>

class Solution;

class Model
{
public:
    class Logger
    {
    public:
        // 接收solution和总距离
        void logSolution(const Solution &sol, double totalDistance);
        // 将记录的解保存到文件
        void saveToFile(const std::string &filename);

    private:
        std::vector<std::pair<Solution, double>> solutions;
    };
    Logger logger;

    int vehicleCount;        // 车数量
    int capacity;            // 车容量
    std::vector<Node> nodes; // 节点(客户)
    std::unordered_map<int, std::unordered_map<int, double>> distanceMatrix;

    Model(int vehicleCount, int capacity);

    void computeDistances();                // 算距离
    Solution initialSolution(Model &model); // 初始解

    static Model loadFromFile(const std::string &filename); // load data

    void print() const; // 新增打印方法
};

#endif
