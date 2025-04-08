#ifndef MODEL_H
#define MODEL_H

#include "Node.h"
#include <vector>
#include <unordered_map>
#include <memory>
// #include "Solution.h"
#include <string>
#include <unordered_set>

class Solution;

// 自定义哈希函数
struct VectorHash
{
    size_t operator()(const std::vector<int> &path) const
    {
        size_t hash = 0;
        for (int node : path)
        {
            hash ^= std::hash<int>()(node) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        }
        return hash;
    }
};

// 自定义比较函数
struct VectorEqual
{
    bool operator()(const std::vector<int> &a, const std::vector<int> &b) const
    {
        return a == b;
    }
};

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
    int fes = 0; // nums of FEs

    double optimalVal;
    int vehicleCount;        // 车数量
    int capacity;            // 车容量
    std::vector<Node> nodes; // 节点(客户)
    std::unordered_map<int, std::unordered_map<int, double>> distanceMatrix;
    std::unordered_map<std::vector<int>, double, VectorHash, VectorEqual> solutionMap;
    Model(int vehicleCount, int capacity);

    void computeDistances();                      // 算距离
    Solution initialSolution(Model &model);       // 初始解
    Solution initialRandomSolution(Model &model); // 随机初始解

    static Model loadFromFile(const std::string &filename); // load data

    void print() const; // 新增打印方法
};

#endif
