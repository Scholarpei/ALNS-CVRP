#include "ALNS.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>
#include "Model.h"

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
        sol.nodes_seq.erase(remove(sol.nodes_seq.begin(), sol.nodes_seq.end(), node), sol.nodes_seq.end());
    }
    sol.update(model); // 更新sol解路径和总长
}

// 从当前解中移除一定比例引起目标函数增幅较大的需求节点
void ALNS::worstRemoval(Solution &sol)
{
    vector<pair<double, int>> deltaCosts;
    Solution tempSol = sol;
    removedNodes.clear();

    // 计算每个节点的影响
    for (const auto &route : sol.routes)
    {
        for (size_t i = 1; i < route.size(); i++)
        {
            int node = route[i];

            auto it = find(tempSol.nodes_seq.begin(), tempSol.nodes_seq.end(), node);
            if (it != tempSol.nodes_seq.end())
            {
                tempSol.nodes_seq.erase(it);
                tempSol.update(model);
                double newCost = tempSol.total_distance;
                double deltaF = sol.total_distance - newCost;

                deltaCosts.emplace_back(deltaF, node);

                tempSol.nodes_seq.insert(it, node);
            }
        }
    }

    // 按照 Δf 由大到小排序（优先移除影响大的）
    sort(deltaCosts.rbegin(), deltaCosts.rend());

    // 选取随机 d（移除数量）
    uniform_int_distribution<int> dist(worst_d_min, worst_d_max);
    int d = dist(gen);

    for (int i = 0; i < min(d, (int)deltaCosts.size()); i++)
    {
        removedNodes.push_back(deltaCosts[i].second);
    }

    for (int node : removedNodes)
    {
        auto it = find(sol.nodes_seq.begin(), sol.nodes_seq.end(), node);
        if (it != sol.nodes_seq.end())
        {
            sol.nodes_seq.erase(it);
        }
    }

    sol.update(model);
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

    sort(demands.rbegin(), demands.rend());

    for (int i = 0; i < min(3, (int)demands.size()); i++)
    {
        removedNodes.push_back(demands[i].second);
    }

    for (int node : removedNodes)
    {
        auto it = find(sol.nodes_seq.begin(), sol.nodes_seq.end(), node);
        if (it != sol.nodes_seq.end())
        {
            sol.nodes_seq.erase(it);
        }
    }

    sol.update(model); // 重新计算 routes 和总长
}

// 随机插入修复
void ALNS::randomInsert(Solution &sol, Model &model)
{
    // 获取需要插入的节点
    vector<int> nodes_to_insert;
    for (int i = 1; i < model.nodes.size(); i++) // 从1开始，因为0是仓库节点
    {
        if (std::find(sol.nodes_seq.begin(), sol.nodes_seq.end(), i) == sol.nodes_seq.end()) // 如果当前解中没有该节点
        {
            nodes_to_insert.push_back(i);
        }
    }

    // 遍历每个路径，随机插入节点
    for (int node : nodes_to_insert)
    {
        // 随机选择一个插入位置（不能是仓库节点的位置）
        uniform_int_distribution<int> pos_dist(1, sol.nodes_seq.size() - 1);
        int pos = pos_dist(gen);

        // 插入节点
        sol.nodes_seq.insert(sol.nodes_seq.begin() + pos, node);
    }

    sol.update(model); // 更新解的路径和总长
}

// 优先执行最小代价的插入操作
void min_cost_Repair(Solution &sol, Model &model)
{
    // 获取需要插入的节点
    vector<int> nodes_to_insert;
    for (int i = 1; i < model.nodes.size(); i++) // 从1开始，因为0是仓库节点
    {
        if (std::find(sol.nodes_seq.begin(), sol.nodes_seq.end(), i) == sol.nodes_seq.end()) // 如果当前解中没有该节点
        {
            nodes_to_insert.push_back(i);
        }
    }
    Solution tempSol = sol;                            // 复制当前解
    vector<vector<double>> node_cost_record;           // node_cost_record 记录的是 vector【需要插入的节点】【最优的插入位置】= 插入节点后cost增加值
    double min_cost = std::numeric_limits<int>::max(); // 创建最小代价的记录
    // 计算每个节点的最优插入位置
    for (size_t r = 0; r < nodes_to_insert.size(); r++) // r记录
    {
        min_cost = std::numeric_limits<int>::max();           // 初始化最小代价的记录
        for (size_t i = 0; i < sol.nodes_seq.size() - 1; i++) // 这里i从0开始是用于记录node_cost_record
        {
            // 计算插入该节点的目标函数变化
            tempSol.nodes_seq.insert(tempSol.nodes_seq.begin() + i, nodes_to_insert[r]);
            tempSol.update(model);                        // 更新tempSol路径和总长
            double newCost = tempSol.total_distance;      // solution类中在解的初始化过程中就会计算一次初始的路径长
            double deltaF = newCost - sol.total_distance; // 注：solution类中的目标变量现改名为total_distance（原obj）
            if (deltaF < min_cost)
            {
                min_cost = deltaF;
                node_cost_record[r][i] = deltaF;
            }
            // 恢复原解，此时不需要更新路径和总长
            tempSol = sol; // 恢复原解
        }
    }
    // 按照插入节点后cost增加值进行从小到大排序
}

// 优先选择最优插入与次优插入代价差距大的节点进行插入操作
void regret_Repair(Solution &sol, Model &model);

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
    Model::Logger logger,
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
    double bestCost = bestSolution.total_distance;

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

            double newCost = newSolution.total_distance;

            // 判断是否接受新解
            if (newCost < currentSolution.total_distance)
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
            else if (newCost - currentSolution.total_distance < T)
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
        logger.logSolution(bestSolution, bestCost);

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
