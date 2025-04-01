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
    const int MAXFEs = 5e4;
    int pu;             // 每iter执行pu次优化
    double rand_d_max;  // 随机破坏程度上限
    double rand_d_min;  // 随机破坏程度下限
    double worst_d_max; // 最坏破坏程度上限
    double worst_d_min; // 最坏破坏程度下限
    int regret_n;       // 次优位置个数
    double r1;          // score if the new solution is the best one found so far.
    double r2;          // score if the new solution improves the current solution.
    double r3;          // score if the new solution does not improve the current solution, but is accepted.
    int fes = 0;        // nums of FEs
    double rho;         // 算子权重衰减factor

    std::vector<double> weights_destroy;     // 摧毁算子权重
    std::vector<int> scores_destroy;         // 摧毁算子奖励得分
    std::vector<int> history_scores_destroy; // 摧毁算子奖励得分总计
    std::vector<int> select_destroy;         // 摧毁算子选中次数/每轮
    std::vector<int> history_select_destroy; // 摧毁算子历史选中总数

    std::vector<double> weights_repair;     // 修复算子权重
    std::vector<int> scores_repair;         // 修复算子奖励得分
    std::vector<int> history_scores_repair; // 修复算子奖励得分总计
    std::vector<int> select_repair;         // 修复算子选中次数/每轮
    std::vector<int> history_select_repair; // 摧毁算子历史选中总数

    std::vector<int> removedNodes;
    std::mt19937 gen; // 随机数生成器

public:
    ALNS(Model &model);
    int selectOperator(const std::vector<double> &weights); // 选择算子

    // destroy strategy
    void randomRemoval(Solution &sol); // 随机移除
    void worstRemoval(Solution &sol);  // 从当前解中移除一定比例引起目标函数增幅较大的需求节点
    void demandBasedRemoval(Solution &sol);
    void proximityRemoval(Solution &sol); //

    // repair strategy:
    void randomInsert(Solution &sol, Model& model); // 随机插入修复
    void min_cost_Repair(Solution &sol, Model& model); // 优先执行最小代价的插入操作
    void regret_Repair(Solution &sol, Model& model); // 优先选择最优插入与次优插入代价差距大的节点进行插入操作

    void resetScore();           // 重置算子得分
    void adaptiveWeightUpdate(); // update weights of destroy and repair
    Solution runALNS(
        double rand_d_min,
        double rand_d_max,
        double worst_d_min,
        double worst_d_max,
        int regret_n,
        double r1,
        double r2,
        double r3,
        double phi,
        double rho = 0.9);
};

#endif
