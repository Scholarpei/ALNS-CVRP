// Solution解类
#include "Solution.h"
#include "Model.h"
#include <iostream>
#include <vector>
#include <utility>
#include <cmath>

using namespace std;

Solution::Solution(vector<int> nodes_seq, Model &model) : nodes_seq(nodes_seq)
{
    pair<int, vector<vector<int>>> split_result = splitRoutes(model); // 调用路径分割函数
    num_vehicle = split_result.first;                                 // 初始化车辆数
    routes = split_result.second;                                     // 初始化路径
    total_distance = evaluateSolution(model);                         // 初始化总路径长
}

void Solution::update(Model &model)
{                                                                     // 这个函数是用来更新路径划分和更新总路径值，每当修改了TSP路径，需要调用一次这个函数来更新
    pair<int, vector<vector<int>>> split_result = splitRoutes(model); // 调用路径分割函数
    num_vehicle = split_result.first;                                 // 更新车辆数
    routes = split_result.second;                                     // 更新化路径
    // printf("Before comp: %lf", total_distance);
    total_distance = evaluateSolution(model); // 更新总路径长
    // printf("After comp: %lf", total_distance);
}

// 路径分割
pair<int, vector<vector<int>>> Solution::splitRoutes(Model &model)
{
    int num_vehicle = 0;                  // 所需车辆数
    vector<vector<int>> vehicle_routes;   // 车辆路径集合
    vector<int> route;                    // 当前车辆的路径
    double remained_cap = model.capacity; // 当前车辆的剩余容量

    int ware = nodes_seq.front(); // 记录仓库的id
    route.push_back(ware);        // 路径的开头是仓库

    vector<int> nodes_seq_to_split = nodes_seq; // 拷贝TSP的解
    // 移除开头元素
    if (!nodes_seq_to_split.empty())
    {
        nodes_seq_to_split.erase(nodes_seq_to_split.begin());
    }

    // 移除末尾元素
    if (!nodes_seq_to_split.empty())
    {
        nodes_seq_to_split.pop_back();
    }

    // 开始分割
    num_vehicle += 1; // 第一辆车
    for (int node_no : nodes_seq_to_split)
    {
        // 检查当前车辆是否有足够的容量来服务该节点
        if (remained_cap - model.nodes[node_no].demand >= 0)
        {
            // 如果有足够容量，将节点加入当前路径
            route.push_back(node_no);
            remained_cap -= model.nodes[node_no].demand;
        }
        else
        {
            // 如果没有足够容量，当前路径结束，开始新的路径
            route.push_back(ware); // 路径的结尾也是仓库
            vehicle_routes.push_back(route);
            route = {ware};                                              // 新路径仍然从仓库开始
            route.push_back(node_no);                                    // push该节点
            num_vehicle += 1;                                            // 需要启用新车时，车辆数加1
            remained_cap = model.capacity - model.nodes[node_no].demand; // 重置剩余容量
        }
    }
    // 确保最后一个节点是仓库
    if (route.back() != ware)
    {
        route.push_back(ware);
    }
    // 添加最后一个路径
    vehicle_routes.push_back(route);

    return {num_vehicle, vehicle_routes};
}

// 计算总行驶距离
double Solution::evaluateSolution(Model &model)
{
    auto it = model.solutionMap.find(nodes_seq);
    if (it != model.solutionMap.end())
    {
        return it->second;
    }

    total_distance = 0;
    for (const auto &route : routes)
    {
        double route_distance = 0.0;
        for (int n = 0; n < route.size() - 1; n++)
        {
            route_distance += model.distanceMatrix[route[n]][route[n + 1]]; // 将路径上相邻节点间的距离计入路径距离
        }
        // printf("one route: %lf ", route_distance);
        total_distance += route_distance; // 将路径距离计入解的总距离
    }
    // printf("total distance compute: %lf\n", total_distance);

    model.solutionMap.insert({nodes_seq, total_distance});
    model.fes++;

    return total_distance;
}

void Solution::printSolutionINFO()
{
    printf("\n---------------------------------\n");
    printf("printSolutionFun:\n");
    printf("Nodes: ");
    for (auto n : nodes_seq)
    {
        printf("%d ", n);
    }
    printf("\n");
    int i = 1;
    for (auto h : routes)
    {
        printf("%d :", i);
        for (auto l : h)
        {
            printf("%d ", l);
        }
        printf("\n");
        i++;
    }

    printf("total distance:%lf\n", total_distance);
    printf("\n---------------------------------\n");
}