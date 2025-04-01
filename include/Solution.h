#ifndef SOLUTION_H
#define SOLUTION_H
#include "Model.h"

class Solution
{
    public:
    vector<int> nodes_seq;
    double total_distance;
    int num_vehicle;
    vector<vector<int>> routes;

    Solution(vector<int> nodes_seq, Model& model) : nodes_seq(nodes_seq){}
    pair<int, vector<vector<int>>> splitRoutes(Model& model);
    double evaluateSolution(Model& model);
    void update(Model& model);

};

#endif