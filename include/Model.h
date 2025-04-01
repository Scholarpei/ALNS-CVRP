#ifndef MODEL_H
#define MODEL_H

#include "Node.h"
#include "Solution.h"
#include <vector>
#include <unordered_map>
#include <string>

class Model
{
public:
    class Logger
    {
    public:
        // 接收solution和总距离
        void logSolution(const Solution &sol, double totalDistance);
        // 将记录的解保存到文件
        void saveToFile(const std::string &filename = "CVRP结果记录");

    private:
        std::vector<std::pair<Solution, double>> solutions;
    };
    Logger logger;

    int vehicleCount;        // 车数量
    int capacity;            // 车容量
    std::vector<Node> nodes; // 节点(客户)
    std::unordered_map<int, std::unordered_map<int, double>> distanceMatrix;

    Solution bestSolution;

    Model(int vehicleCount, int capacity);

    void computeDistances();                       // 算距离
    Solution Model::initialSolution(Model &model); // 初始解

    static Model loadFromFile(const std::string &filename); // load data

    void print() const; // 新增打印方法
};

#endif
