#include "ALNS.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

using namespace std;

ALNS::ALNS(Model &model) : model(model), gen(random_device{}())
{
    weights_destroy = {1.0, 1.0, 1.0}; // 破坏算子的初始权重
    weights_repair = {1.0, 1.0, 1.0};  // 修复算子的初始权重
    scores_destroy = {0, 0, 0};
    scores_repair = {0, 0, 0};
}

// 随机移除
void ALNS::randomRemoval(Solution &sol)
{
    
}

// 基于距离移除
void ALNS::proximityRemoval(Solution &sol)
{
    
}

// 需求集群移除
void ALNS::demandBasedRemoval(Solution &sol)
{
    
}

// 选择算子 type = destroy or repair
int ALNS::selectOperator(const std::vector<double> &weights, string type)
{
    std::discrete_distribution<int> dist(weights.begin(), weights.end());
    std::uniform_int_distribution<int> dist_destroy(1, 3); // 这里3对应destroy strategy个数
    std::uniform_int_distribution<int> dist_repair(1, 1);  // 对应repair strategy个数
    return (type == "destroy") ? (dist_destroy(gen)) : (dist_repair(gen));
}

// update weight
void ALNS::adaptiveWeightUpdate()
{
    double w1 = 10.0, w2 = 5.0, w3 = 1.0;
    for (size_t i = 0; i < scores_destroy.size(); i++)
    {
        weights_destroy[i] = decayFactor * weights_destroy[i] + w1 * scores_destroy[i];
    }
    for (size_t i = 0; i < scores_repair.size(); i++)
    {
        weights_repair[i] = decayFactor * weights_repair[i] + w1 * scores_repair[i];
    }
    std::fill(scores_destroy.begin(), scores_destroy.end(), 0);
    std::fill(scores_repair.begin(), scores_repair.end(), 0);
}

// run the algo
Solution ALNS::runALNS(int iterations)
{
    Solution bestSolution = model.bestSolution;
    Solution currentSolution = bestSolution;
    double bestCost = model.evaluateSolution(bestSolution);

    for (int iter = 0;; iter++)
    {
        Solution newSolution = currentSolution;

        int destroyIdx = selectOperator(weights_destroy, "destroy");
        int repairIdx = selectOperator(weights_repair, "repair");

        if (destroyIdx == 0)
            randomRemoval(newSolution);
        else if (destroyIdx == 1)
            proximityRemoval(newSolution);
        else if (destroyIdx == 2)
            demandBasedRemoval(newSolution);

        // to be written
        // call repair operator

        double newCost = model.evaluateSolution(newSolution);

        if (newCost < bestCost)
        {
            bestSolution = newSolution;
            bestCost = newCost;
            scores_destroy[destroyIdx] += 10;
            scores_repair[repairIdx] += 10;
        }
        else
        {
            scores_destroy[destroyIdx] += 1;
            scores_repair[repairIdx] += 1;
        }

        if (iter % 10 == 0)
        {
            adaptiveWeightUpdate();
        }

        printf("iter: %d bestCost: %d\n", iter, bestCost);
    }

    return bestSolution;
}
