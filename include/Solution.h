#ifndef SOLUTION_H
#define SOLUTION_H
#include <vector>

class Model;

class Solution
{
public:
    std::vector<int> nodes_seq;
    double total_distance;
    int num_vehicle;
    std::vector<std::vector<int>> routes;

    Solution(std::vector<int> nodes_seq, Model &model);
    std::pair<int, std::vector<std::vector<int>>> splitRoutes(Model &model);
    double evaluateSolution(Model &model);
    double evaluateSingleRoute(Model &model,const std::vector<int> &route); // 评估单独的路径
    void update(Model &model);
    void printSolutionINFO();
};

#endif