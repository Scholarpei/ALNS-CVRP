#include "ALNS.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>
#include "Model.h"
#include <sstream>
#include <string>
#include <random>

using namespace std;

ALNS::ALNS(Model &model, int SEED) : model(model), gen(random_device{}())
{
    std::random_device rd;
    gen.seed(SEED);                    // 可以加上时间作为扰动
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
void ALNS::randomRepair(Solution &sol, Model &model)
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

// 最小cost贪婪修复算子
void ALNS::min_cost_Repair(Solution &sol, Model &model)
{
    // 获取需要插入的节点
    vector<int> nodes_to_insert;
    for (int i = 1; i < model.nodes.size(); i++)
    { // 从1开始，因为0是仓库节点
        if (std::find(sol.nodes_seq.begin(), sol.nodes_seq.end(), i) == sol.nodes_seq.end())
        { // 如果当前解中没有该节点
            nodes_to_insert.push_back(i);
        }
    }

    // 初始化记录插入代价的二维向量
    vector<vector<double>> node_cost_record(nodes_to_insert.size(), vector<double>(sol.nodes_seq.size(), std::numeric_limits<double>::max()));

    // 每次选择最优的插入操作，直到全部插入
    while (!nodes_to_insert.empty())
    {

        // 复原矩阵，更新矩阵大小（每次循环中由于nodes_to_insert和nodes_seq元素个数不同，矩阵大小不同）
        node_cost_record = vector<vector<double>>(nodes_to_insert.size(), vector<double>(sol.nodes_seq.size(), std::numeric_limits<double>::max()));

        for (size_t node_idx = 0; node_idx < nodes_to_insert.size(); node_idx++)
        {
            Solution tempSol = sol; // 深拷贝当前解
            if (sol.nodes_seq.size() <= 2)
                continue; // 无法插入
            for (size_t insert_pos = 1; insert_pos < sol.nodes_seq.size(); insert_pos++)
            { // 插入位置应为 【1 - sol.nodes_seq.size() - 1】，不能插入到两侧
                // 计算插入该节点的目标函数变化
                tempSol.nodes_seq.insert(tempSol.nodes_seq.begin() + insert_pos, nodes_to_insert[node_idx]);
                tempSol.update(model);                        // 更新tempSol路径和总长
                double newCost = tempSol.total_distance;      // 获取新的总代价
                double deltaF = newCost - sol.total_distance; // 计算代价变化

                // 更新记录
                if (deltaF < node_cost_record[node_idx][insert_pos])
                {
                    node_cost_record[node_idx][insert_pos] = deltaF;
                }

                // 恢复原解
                tempSol = sol; // 深拷贝恢复
            }
        }

        // 按照插入节点后cost增加值找到最小的项
        double min_cost = std::numeric_limits<double>::max();
        int min_node_idx = -1;
        int min_insert_pos = -1;
        for (size_t node_idx = 0; node_idx < nodes_to_insert.size(); node_idx++)
        {
            for (size_t insert_pos = 1; insert_pos < sol.nodes_seq.size(); insert_pos++)
            { // 插入位置应为 【1 - sol.nodes_seq.size() - 1】，不能插入到两侧
                if (node_cost_record[node_idx][insert_pos] < min_cost)
                {
                    min_cost = node_cost_record[node_idx][insert_pos];
                    min_node_idx = node_idx;
                    min_insert_pos = insert_pos;
                }
            }
        }

        // 如果找到了最小代价增加的节点，进行插入操作
        if (min_node_idx != -1)
        {
            int min_cost_node = nodes_to_insert[min_node_idx];
            sol.nodes_seq.insert(sol.nodes_seq.begin() + min_insert_pos, min_cost_node); // 插入位置应为 【1 - sol.nodes_seq.size() - 1】，不能插入到两侧
            sol.update(model);                                                           // 更新sol路径和总长
            nodes_to_insert.erase(nodes_to_insert.begin() + min_node_idx);               // 移除已插入的节点
        }
    }
}

// 遗憾修复算子
void ALNS::regret_Repair(Solution &sol, Model &model)
{
    // 获取需要插入的节点
    vector<int> nodes_to_insert;
    for (int i = 1; i < model.nodes.size(); i++)
    { // 从1开始，因为0是仓库节点
        if (std::find(sol.nodes_seq.begin(), sol.nodes_seq.end(), i) == sol.nodes_seq.end())
        { // 如果当前解中没有该节点
            nodes_to_insert.push_back(i);
        }
    }

    // 每次选择最优的插入操作，直到全部插入
    while (!nodes_to_insert.empty())
    {
        // 初始化记录插入代价的二维向量
        vector<vector<double>> node_cost_record(nodes_to_insert.size(), vector<double>(sol.nodes_seq.size(), std::numeric_limits<double>::max()));

        // 计算每个节点的插入代价
        for (size_t node_idx = 0; node_idx < nodes_to_insert.size(); node_idx++)
        {
            Solution tempSol = sol; // 深拷贝当前解
            for (size_t insert_pos = 1; insert_pos < sol.nodes_seq.size(); insert_pos++)
            { // 插入位置应为 [1, sol.nodes_seq.size() - 1]
                // 计算插入该节点的目标函数变化
                tempSol.nodes_seq.insert(tempSol.nodes_seq.begin() + insert_pos, nodes_to_insert[node_idx]);
                tempSol.update(model);                        // 更新tempSol路径和总长
                double newCost = tempSol.total_distance;      // 获取新的总代价
                double deltaF = newCost - sol.total_distance; // 计算代价变化

                // 更新记录
                if (deltaF < node_cost_record[node_idx][insert_pos])
                {
                    node_cost_record[node_idx][insert_pos] = deltaF;
                }

                // 恢复原解
                tempSol = sol; // 深拷贝恢复
            }
        }

        // 找到每个节点的最优和次优插入位置
        struct NodeInsertInfo
        {
            int node_idx;
            int best_insert_pos;
            vector<double> top_costs; // 存储最优、次优、第三优、第四优的代价
        };

        vector<NodeInsertInfo> node_insert_info(nodes_to_insert.size());

        for (size_t node_idx = 0; node_idx < nodes_to_insert.size(); node_idx++)
        {
            vector<pair<double, int>> costs;
            for (size_t insert_pos = 1; insert_pos < sol.nodes_seq.size(); insert_pos++)
            {
                costs.push_back({node_cost_record[node_idx][insert_pos], insert_pos});
            } // cost值，要插入的位置

            // 按代价排序
            sort(costs.begin(), costs.end());

            // 获取最优、次优、第三优、第四优的代价
            node_insert_info[node_idx].node_idx = node_idx;
            node_insert_info[node_idx].best_insert_pos = costs[0].second;
            node_insert_info[node_idx].top_costs.resize(4, std::numeric_limits<double>::max());
            for (size_t i = 0; i < std::min(static_cast<size_t>(4), costs.size()); i++)
            {
                node_insert_info[node_idx].top_costs[i] = costs[i].first;
            }
        }

        // 找到最优插入位置与次优、第三优、第四优代价差距之和最大的节点
        int best_node_idx = -1;
        double max_regret = -std::numeric_limits<double>::max();

        for (const auto &info : node_insert_info)
        {
            double regret = 0;
            for (size_t i = 1; i < info.top_costs.size(); i++)
            {
                if (info.top_costs[i] != std::numeric_limits<double>::max())
                {
                    regret += info.top_costs[i] - info.top_costs[0]; // 累加差距
                }
            }
            if (regret > max_regret)
            {
                max_regret = regret;
                best_node_idx = info.node_idx;
            }
        }

        // 如果找到了最优节点，进行插入操作
        if (best_node_idx != -1)
        {
            int best_node = nodes_to_insert[best_node_idx];
            int best_insert_pos = node_insert_info[best_node_idx].best_insert_pos;

            sol.nodes_seq.insert(sol.nodes_seq.begin() + best_insert_pos, best_node);
            sol.update(model);                                              // 更新sol路径和总长
            nodes_to_insert.erase(nodes_to_insert.begin() + best_node_idx); // 移除已插入的节点
        }
    }
}

double evaluateSingleRoute(Model &model, const std::vector<int> &route)
{
    auto it = model.solutionMap.find(route);
    if (it != model.solutionMap.end())
    {
        return it->second;
    } // 已经有了的直接返回不计算不加fes
    double route_distance = 0.0;
    for (int n = 0; n < route.size() - 1; n++)
    {
        route_distance += model.distanceMatrix[route[n]][route[n + 1]]; // 将路径上相邻节点间的距离计入路径距离
    }
    model.solutionMap.insert({route, route_distance});
    model.fes++;

    return route_distance;
}

// (not used)
std::vector<int> ALNS::twoOpt(Model &model, const std::vector<int> &route)
{
    int n = route.size();
    if (n < 4)
        return route; // 至少4个点（含起点终点）

    std::vector<int> best_route = route;
    double best_cost = evaluateSingleRoute(model, route);

    bool improved = true;

    while (improved)
    {
        improved = false;
        for (int i = 1; i < n - 2; ++i) // 不动起点
        {
            for (int j = i + 1; j < n - 1; ++j) // 不动终点
            {
                std::vector<int> new_route = best_route;
                std::reverse(new_route.begin() + i, new_route.begin() + j + 1);
                double new_cost = evaluateSingleRoute(model, new_route);

                if (new_cost < best_cost)
                {
                    best_cost = new_cost;
                    best_route = new_route;
                    improved = true;
                }
            }
        }
    }

    return best_route;
}

// 对每一条路径应用 3-opt 局部搜索 (not used)
std::vector<int> ALNS::threeOpt(const std::vector<int> &route)
{
    int n = route.size();
    if (n < 6)
        return route; // 3-opt 至少需要6个点

    std::vector<int> best_route = route;
    double best_cost = evaluateSingleRoute(model, route);

    bool improved = true;

    while (improved)
    {
        improved = false;

        for (int i = 1; i < n - 4; i++)
        {
            for (int j = i + 1; j < n - 2; j++)
            {
                for (int k = j + 1; k < n; k++)
                {
                    std::vector<std::vector<int>> candidates;

                    // 三段路径
                    std::vector<int> A(route.begin(), route.begin() + i);
                    std::vector<int> B(route.begin() + i, route.begin() + j);
                    std::vector<int> C(route.begin() + j, route.begin() + k);
                    std::vector<int> D(route.begin() + k, route.end());

                    // 生成不同排列方式
                    candidates.push_back(concat(A, B, C, D)); // 原顺序
                    candidates.push_back(concat(A, reverse(B), C, D));
                    candidates.push_back(concat(A, B, reverse(C), D));
                    candidates.push_back(concat(A, reverse(B), reverse(C), D));
                    candidates.push_back(concat(A, C, B, D));
                    candidates.push_back(concat(A, reverse(C), B, D));
                    candidates.push_back(concat(A, C, reverse(B), D));
                    candidates.push_back(concat(A, reverse(C), reverse(B), D));

                    for (auto &cand : candidates)
                    {
                        double cand_cost = evaluateSingleRoute(model, cand);
                        if (cand_cost < best_cost)
                        {
                            best_cost = cand_cost;
                            best_route = cand;
                            improved = true;
                        }
                    }
                }
            }
        }
    }

    return best_route;
}

// 用于拼接多个路径段
std::vector<int> ALNS::concat(const std::vector<int> &a, const std::vector<int> &b,
                              const std::vector<int> &c, const std::vector<int> &d)
{
    std::vector<int> result = a;
    result.insert(result.end(), b.begin(), b.end());
    result.insert(result.end(), c.begin(), c.end());
    result.insert(result.end(), d.begin(), d.end());
    return result;
}

// 反转路径段
std::vector<int> ALNS::reverse(const std::vector<int> &vec)
{
    std::vector<int> reversed = vec;
    std::reverse(reversed.begin(), reversed.end());
    return reversed;
}

// 选择算子 type = destroy or repair
int ALNS::selectOperator(const std::vector<long double> &weights)
{
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
    double rho)
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

    // Solution bestSolution = model.initialRandomSolution(model);
    // Solution bestSolution = model.initialSolution(model);
    Solution bestSolution = model.initialBestOfGreedyAndRandom(200);

    // model.print();
    // bestSolution.printSolutionINFO();

    Solution currentSolution = bestSolution;
    double bestCost = bestSolution.total_distance;

    // printf("%Lf\n", weights_destroy[0]);
    int iter;

    for (iter = 0; model.fes <= 50000; iter++)
    {
        if (iter > 500)
            if (weights_destroy[0] < 0.3)
                // weights_destroy[0] = 1;
                weights_destroy[0] = max(weights_destroy[1], weights_destroy[2]);
        if (iter > 2e5)
            break;

        Solution newSolution = currentSolution;
        double T = bestCost * 0.2;
        resetScore();

        double newCost = newSolution.total_distance;

        for (int k = 0; k < pu; k++)
        {
            // choose
            int destroyIdx = selectOperator(weights_destroy);
            int repairIdx = selectOperator(weights_repair);
            select_destroy[destroyIdx]++;
            select_repair[repairIdx]++;

            // printf("destory: %d repair %d \n", destroyIdx, repairIdx);

            if (destroyIdx == 0)
                randomRemoval(newSolution);
            else if (destroyIdx == 1)
                worstRemoval(newSolution);
            else if (destroyIdx == 2)
                demandBasedRemoval(newSolution);

            if (repairIdx == 0)
                randomRepair(newSolution, model);
            else if (repairIdx == 1)
                min_cost_Repair(newSolution, model);
            else if (repairIdx == 2)
                regret_Repair(newSolution, model);

            // // 对每条路径应用 2-opt
            // for (auto &route : newSolution.routes)
            // {
            //     route = twoOpt(model, route); // 优化路径
            // }

            // 对每条路径应用 3-opt
            // for (auto &route : newSolution.routes)
            // {
            //     route = threeOpt(route); // 优化每条路径
            // }

            // // 重新评估整个解的目标值
            newSolution.update(model);

            newCost = newSolution.total_distance;

            // printf("newCost : %lf\n", newCost);
            // 将新解与map中的解比较，更新fes
            // checkAndAddSolution(model.solutionSet, newSolution.nodes_seq, model.fes);

            // 判断是否接受新解
            if (newCost < currentSolution.total_distance)
            {
                currentSolution = newSolution;
                if (newCost < bestCost)
                {
                    // printf("update Solution!\n");
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
            // printf("newCost :%lf\n", newCost);

            // 退火
            T *= phi;

            // 记录最优解
            // // history_best_obj.push_back(bestCost);
            // printf("random destroy weight: %Lf\tselected: %d\tscore: %lf\n",
            //        weights_destroy[0], history_select_destroy[0], history_scores_destroy[0]);
        }
        adaptiveWeightUpdate();
        logger.logSolution(bestSolution, bestCost);

        // printf("iter: %d best
        // printf("iter %d Cost: %lf fes: %d \n", iter, bestCost, model.fes);
    }

    printf("Best cost: %lf Use iter: %d\n", bestCost, iter);

    printf("random destroy weight: %Lf\tselected: %d\tscore: %lf\n",
           weights_destroy[0], history_select_destroy[0], history_scores_destroy[0]);
    printf("worse destroy weight: %Lf\tselected: %d\tscore: %lf\n",
           weights_destroy[1], history_select_destroy[1], history_scores_destroy[1]);
    printf("Demand destroy weight: %Lf\tselected: %d\tscore: %lf\n",
           weights_destroy[2], history_select_destroy[2], history_scores_destroy[2]);

    printf("random repair weight: %Lf\tselected: %d\tscore: %lf\n",
           weights_repair[0], history_select_repair[0], history_scores_repair[0]);
    printf("greedy repair weight: %Lf\tselected: %d\tscore: %lf\n",
           weights_repair[1], history_select_repair[1], history_scores_repair[1]);
    printf("regret repair weight: %Lf\tselected: %d\tscore: %lf\n",
           weights_repair[2], history_select_repair[2], history_scores_repair[2]);

    return bestSolution;
}
