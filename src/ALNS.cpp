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
    history_scores_destroy = {0, 0, 0};
    history_scores_repair = {0, 0, 0};
    select_destroy = {0, 0, 0};
    select_repair = {0, 0, 0};
    history_select_destroy = {0, 0, 0};
    history_select_repair = {0, 0, 0};

    // rand_d_min...?
}

// 随机移除
void ALNS::randomRemoval(Solution &sol)
{
    uniform_real_distribution<> dis(rand_d_min, rand_d_max);
    double d = dis(gen); // 随机的rand_d

    int removeCount = sol.routes.size() * d;
    uniform_int_distribution<int> dist(1, model.nodes.size() - 1);
    removedNodes.clear();

    for (int i = 0; i < removeCount; i++)
    {
        int node = dist(gen);
        removedNodes.push_back(node);

        for (auto &route : sol.routes)
        {
            route.erase(remove(route.begin(), route.end(), node), route.end());
        }
    }
}

// 从当前解中移除一定比例引起目标函数增幅较大的需求节点
void ALNS::worstRemoval(Solution &sol)
{
    vector<pair<double, int>> deltaCosts;
    Solution tempSol = sol; // 复制当前解
    removedNodes.clear();

    // 计算每个节点的影响
    for (size_t r = 0; r < sol.routes.size(); r++)
    {
        for (size_t i = 1; i < sol.routes[r].size(); i++)
        { // 避免移除起点（仓库）
            int node = sol.routes[r][i];

            // 计算移除该节点的目标函数变化
            tempSol.routes[r].erase(tempSol.routes[r].begin() + i);
            double newCost = model.evaluateSolution(tempSol);
            double deltaF = sol.obj - newCost;

            // 记录 (影响, 节点)
            deltaCosts.emplace_back(deltaF, node);

            // 恢复原解
            tempSol.routes[r].insert(tempSol.routes[r].begin() + i, node);
        }
    }

    // 按照 Δf 由大到小排序（优先移除影响小的）
    sort(deltaCosts.begin(), deltaCosts.end(), greater<>());

    // 选取随机 d（移除数量）
    uniform_int_distribution<int> dist(worst_d_min, worst_d_max);
    int d = dist(gen);

    for (int i = 0; i < min(d, (int)deltaCosts.size()); i++)
    {
        removedNodes.push_back(deltaCosts[i].second);
    }

    // 从解中移除选中的节点
    for (auto &route : sol.routes)
    {
        for (int node : removedNodes)
        {
            route.erase(remove(route.begin(), route.end(), node), route.end());
        }
    }
}

// 基于距离移除
void ALNS::proximityRemoval(Solution &sol)
{
    // uniform_int_distribution<int> dist(1, model.nodes.size() - 1);
    // int seedNode = dist(gen);
    // removedNodes.clear();
    // removedNodes.push_back(seedNode);

    // vector<pair<double, int>> distances;
    // for (size_t i = 1; i < model.nodes.size(); i++)
    // {
    //     if (i != seedNode)
    //     {
    //         double d = model.distanceMatrix[seedNode][i];
    //         distances.emplace_back(d, i);
    //     }
    // }
    // sort(distances.begin(), distances.end());

    // for (int i = 0; i < min(3, (int)distances.size()); i++)
    // {
    //     removedNodes.push_back(distances[i].second);
    // }

    // for (auto &route : sol.routes)
    // {
    //     for (int node : removedNodes)
    //     {
    //         route.erase(remove(route.begin(), route.end(), node), route.end());
    //     }
    // }
}

// 需求集群移除
void ALNS::demandBasedRemoval(Solution &sol)
{
    removedNodes.clear();
    vector<pair<int, int>> demands;
    for (size_t i = 1; i < model.nodes.size(); i++)
    {
        demands.emplace_back(model.nodes[i].demand, i);
    }
    sort(demands.begin(), demands.end(), greater<>());

    for (int i = 0; i < min(3, (int)demands.size()); i++)
    {
        removedNodes.push_back(demands[i].second);
    }

    for (auto &route : sol.routes)
    {
        for (int node : removedNodes)
        {
            route.erase(remove(route.begin(), route.end(), node), route.end());
        }
    }
}

// 选择算子 type = destroy or repair
int ALNS::selectOperator(const std::vector<double> &weights)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::discrete_distribution<int> dist(weights.begin(), weights.end());
    return dist(gen);
}

void ALNS::resetScore()
{
    scores_destroy = {0, 0, 0};
    select_destroy = {0, 0, 0};

    scores_repair = {0, 0, 0};
    select_repair = {0, 0, 0};
}

// update weight
void ALNS::adaptiveWeightUpdate()
{
    // 更新摧毁算子权重
    for (size_t i = 0; i < weights_destroy.size(); i++)
    {
        if (select_destroy[i] > 0)
        {
            weights_destroy[i] = weights_destroy[i] * (1 - rho) + rho * scores_destroy[i] / select_destroy[i];
        }
        else
        {
            weights_destroy[i] *= (1 - rho);
        }
    }

    // 更新修复算子权重
    for (size_t i = 0; i < weights_repair.size(); i++)
    {
        if (select_repair[i] > 0)
        {
            weights_repair[i] = weights_repair[i] * (1 - rho) + rho * scores_repair[i] / select_repair[i];
        }
        else
        {
            weights_repair[i] *= (1 - rho);
        }
    }

    // 记录历史选择次数和得分
    for (size_t i = 0; i < select_destroy.size(); i++)
    {
        history_select_destroy[i] += select_destroy[i];
        history_scores_destroy[i] += scores_destroy[i];
    }

    for (size_t i = 0; i < select_repair.size(); i++)
    {
        history_select_repair[i] += select_repair[i];
        history_scores_repair[i] += scores_repair[i];
    }
}

// run the algo
//     :param rand_d_max: max degree of random destruction
//     :param rand_d_min: min degree of random destruction
//     :param worst_d_max: max degree of worst destruction
//     :param worst_d_min: min degree of worst destruction
//     :param regret_n:  n next cheapest insertions
//     :param r1: score if the new solution is the best one found so far.
//     :param r2: score if the new solution improves the current solution.
//     :param r3: score if the new solution does not improve the current solution, but is accepted.
//     :param rho: reaction factor of action weight
//     :param phi: the reduction factor of threshold
//     :param epochs: Iterations
//     :param pu: the frequency of weight adjustment
//     :param v_cap: Vehicle capacity
Solution ALNS::runALNS(
    double rand_d_min,
    double rand_d_max,
    double worst_d_min,
    double worst_d_max,
    int regret_n,
    double r1,
    double r2,
    double r3,
    double phi,
    double rho = 0.9)
{
    this->rand_d_min = rand_d_min;
    this->rand_d_max = rand_d_max;
    this->worst_d_max = worst_d_max;
    this->worst_d_min = worst_d_min;
    this->regret_n = regret_n;
    this->r1 = r1;
    this->r2 = r2;
    this->r3 = r3;
    this->rho = rho;

    Solution bestSolution = model.bestSolution;
    Solution currentSolution = bestSolution;
    double bestCost = model.evaluateSolution(bestSolution);

    for (int iter = 0;; iter++)
    {
        Solution newSolution = currentSolution;
        double T = bestCost * 0.2;
        resetScore();

        for (int k = 0; k < pu; k++)
        {
            // choose
            int destroyIdx = selectOperator(weights_destroy);
            int repairIdx = selectOperator(weights_repair);
            select_destroy[destroyIdx]++;
            select_repair[repairIdx]++;

            if (destroyIdx == 0)
                randomRemoval(newSolution);
            else if (destroyIdx == 1)
                worstRemoval(newSolution);
            else if (destroyIdx == 2)
                demandBasedRemoval(newSolution);

            // to be written
            // call repair operator

            double newCost = model.evaluateSolution(newSolution);

            // 判断是否接受新解
            if (newCost < currentSolution.obj)
            {
                currentSolution = newSolution;
                if (newCost < bestCost)
                {
                    bestSolution = newSolution;
                    bestCost = newCost;
                    scores_destroy[destroyIdx] += r1;
                    scores_repair[repairIdx] += r1;
                }
                else
                {
                    scores_destroy[destroyIdx] += r2;
                    scores_repair[repairIdx] += r2;
                }
            }
            else if (newCost - currentSolution.obj < T)
            {
                currentSolution = newSolution;
                scores_destroy[destroyIdx] += r3;
                scores_repair[repairIdx] += r3;
            }

            // 退火
            T *= phi;

            // 记录最优解
            // history_best_obj.push_back(bestCost);
        }
        adaptiveWeightUpdate();

        printf("iter: %d bestCost: %d\n", iter, bestCost);
    }

    printf("random destroy weight: %.3f\tselected: %d\tscore: %.3f\n",
           weights_destroy[0], history_select_destroy[0], history_scores_destroy[0]);
    printf("worse destroy weight: %.3f\tselected: %d\tscore: %.3f\n",
           weights_destroy[1], history_select_destroy[1], history_scores_destroy[1]);
    printf("random repair weight: %.3f\tselected: %d\tscore: %.3f\n",
           weights_repair[0], history_select_repair[0], history_scores_repair[0]);
    printf("greedy repair weight: %.3f\tselected: %d\tscore: %.3f\n",
           weights_repair[1], history_select_repair[1], history_scores_repair[1]);
    printf("regret repair weight: %.3f\tselected: %d\tscore: %.3f\n",
           weights_repair[2], history_select_repair[2], history_scores_repair[2]);

    return bestSolution;
}
