#ifndef ALNS_H
#define ALNS_H

#include "Solution.h"
#include "Model.h"
#include <vector>
#include <random>

class ALNS
{
private:
    Model &model;
    std::vector<double> weights_destroy; // 摧毁算子权重
    std::vector<double> weights_repair;  // 修复算子权重
    std::vector<int> scores_destroy;     //
    std::vector<int> scores_repair;
    std::vector<int> removedNodes;
    std::mt19937 gen; // 随机数生成器
    int fes = 0;      // nums of FEs

    double decayFactor = 0.9;

public:
    ALNS(Model &model);
    int selectOperator(const std::vector<double> &weights, std::string type); // 选择算子

    // destroy strategy
    void randomRemoval(Solution &sol);    // 随机移除
    void proximityRemoval(Solution &sol); //
    void demandBasedRemoval(Solution &sol);

    // repair strategy:

    void adaptiveWeightUpdate(); // update weights of destroy and repair
    Solution runALNS(int iterations);
};

#endif
