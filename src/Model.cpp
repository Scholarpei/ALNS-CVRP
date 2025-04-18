// CVRP 模型
#include "Model.h"
#include "Solution.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <algorithm>
#include <random>
#include <regex>

using namespace std;

void Model::Logger::logSolution(const Solution &sol, double totalDistance)
{
    // 打包并添加到一个容器中
    solutions.emplace_back(sol, totalDistance);
}

void Model::Logger::saveToFile(const std::string &filename = "result")
{
    std::ofstream out(filename);
    // 遍历solution容器中的每个记录
    for (const auto &[sol, dist] : solutions)
    {
        out << "Total Distance: " << dist << "\n";
        out << "Routes:\n";
        // 规定路线输出格式，[a,b,c,...]
        for (const auto &route : sol.routes)
        {
            out << "[";
            for (size_t i = 0; i < route.size(); ++i)
            {
                out << route[i];
                if (i < route.size() - 1)
                    out << ", ";
            }
            out << "]\n";
        }
        // 划分不同组的解
        out << "--------------------\n";
    }
}

Model::Model(int vehicleCount, int capacity)
    : vehicleCount(vehicleCount), capacity(capacity), logger()
{
}

void Model::computeDistances()
{
    const double self2self = 1e9; // 定义节点自身距离

    distanceMatrix.clear();
    for (const auto &from : nodes)
    {
        for (const auto &to : nodes)
        {
            // 如果是自身节点，设为INF
            if (from.id == to.id)
            {
                distanceMatrix[from.id][to.id] = self2self;
            }
            else
            {
                // 正常计算欧式距离
                double dx = from.x - to.x;
                double dy = from.y - to.y;
                distanceMatrix[from.id][to.id] = std::sqrt(dx * dx + dy * dy);
            }
        }
    }
}

void Model::print() const
{
    // 基本信息
    std::cout << "\n========== Model Status ==========\n";
    std::cout << "Vehicles: " << vehicleCount
              << " | Capacity: " << capacity
              << " | Nodes: " << nodes.size() << "\n\n";

    // 节点列表（按ID排序）
    std::vector<Node> sortedNodes = nodes;
    std::sort(sortedNodes.begin(), sortedNodes.end(),
              [](const Node &a, const Node &b)
              { return a.id < b.id; });

    std::cout << "=== Node List (ID | Coordinates | Demand) ===\n";
    std::cout << std::left
              << std::setw(6) << "ID"
              << std::setw(15) << "Coordinates"
              << "Demand\n";

    for (const auto &node : sortedNodes)
    {
        std::cout << std::setw(6) << node.id
                  << "(" << std::setw(4) << node.x
                  << "," << std::setw(4) << node.y << ")"
                  << std::setw(10) << ""
                  << std::setw(4) << node.demand << "\n";
    }

    // 距离矩阵样本
    std::cout << "\n=== Distance Matrix Sample ===\n";
    if (distanceMatrix.empty())
    {
        std::cout << "Distance matrix not computed!\n";
        return;
    }

    // 打印前5个节点间的距离
    const int sampleSize = std::min(5, static_cast<int>(nodes.size()));
    std::cout << std::fixed << std::setprecision(2);

    // 表头
    std::cout << std::setw(8) << "From\\To";
    for (int j = 0; j < sampleSize; ++j)
    {
        std::cout << std::setw(8) << sortedNodes[j].id;
    }
    std::cout << "\n";

    // 表格内容
    for (int i = 0; i < sampleSize; ++i)
    {
        std::cout << std::setw(8) << sortedNodes[i].id;
        for (int j = 0; j < sampleSize; ++j)
        {
            auto from = sortedNodes[i].id;
            auto to = sortedNodes[j].id;
            std::cout << std::setw(8) << distanceMatrix.at(from).at(to);
        }
        std::cout << "\n";
    }

    // 仓库节点验证
    auto depot = std::find_if(nodes.begin(), nodes.end(),
                              [](const Node &n)
                              { return n.demand == 0; });
    if (depot != nodes.end())
    {
        std::cout << "\n[Depot] ID: " << depot->id
                  << " | Location: (" << depot->x << ", " << depot->y << ")\n";
    }
}

Solution Model::initialRandomSolution(Model &model)
{
    // 确保距离矩阵已计算
    if (distanceMatrix.empty())
    {
        computeDistances();
    }

    vector<int> init_route;
    int num_nodes = nodes.size();

    // 创建一个城市编号列表（除了仓库 0）
    for (int i = 1; i < num_nodes; ++i)
    {
        init_route.push_back(i);
    }

    // 使用随机数引擎打乱节点顺序
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(init_route.begin(), init_route.end(), g);

    // 加入起点（仓库）作为开始和结束
    init_route.insert(init_route.begin(), 0);
    init_route.push_back(0);

    Solution rand_sol(init_route, model);
    return rand_sol;
}

// 距离贪婪算法求TSP初始解
Solution Model::initialSolution(Model &model)
{
    // 确保距离矩阵已计算
    if (distanceMatrix.empty())
    {
        computeDistances();
    }
    vector<int> init_route;
    init_route.push_back(0); // 起点为仓库

    int current = 0; // 定义现节点
    int next = 0;    // 定义下一个节点
    vector<int> isvisit(nodes.size());
    for (auto n : isvisit)
    {
        n = 0; // 初始化为未访问
    }
    isvisit[0] = 1; // 仓库设置已访问
    for (int n = 1; n < nodes.size(); n++)
    {
        int min_dist = std::numeric_limits<int>::max(); // 初始化cost
        for (int d = 1; d < nodes.size(); d++)
        {
            if (!isvisit[d])
            {
                if (distanceMatrix[current][d] < min_dist)
                {
                    min_dist = distanceMatrix[current][d];
                    next = d;
                }
            }
        }
        init_route.push_back(next); // 推入距离最小的节点
        isvisit[next] = true;
        current = next; // 更新current
    }
    init_route.push_back(0); // 结尾是仓库

    // for (auto n : init_route)
    // {
    //     printf("%d ", n);
    // }

    Solution init_sol(init_route, model);
    return init_sol;
}

Solution Model::initialBestOfGreedyAndRandom(int num_random)
{
    // 确保距离矩阵已计算
    if (distanceMatrix.empty())
    {
        computeDistances();
    }

    // 用贪婪解初始化
    Solution best_sol = initialSolution(*this);
    double best_cost = best_sol.total_distance;

    // 生成 num_random 个随机解
    for (int i = 0; i < num_random; ++i)
    {
        Solution rand_sol = initialRandomSolution(*this);
        double rand_cost = rand_sol.total_distance;
        if (rand_cost < best_cost)
        {
            best_sol = rand_sol;
            best_cost = rand_cost;
        }
    }

    return best_sol;
}

Model Model::loadFromFile(const string &path, const string &filename)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        throw std::runtime_error("无法打开文件: " + filename);
    }

    std::string line;
    std::string name, type, edgeWeightType;
    int dimension = 0, capacity = 0, vehicleCount = 0;
    std::vector<Node> nodes;
    std::unordered_map<int, size_t> idToIndex;
    int depotId = -1;

    double optimalV = 0;

    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        std::string key;
        if (!(iss >> key))
            continue;

        if (key == "NAME")
        {
            std::string colon, nameStr;
            iss >> colon >> nameStr;
            // 从名称中提取车辆数量，例如"A-n32-k5"中的k5
            size_t kPos = nameStr.find_last_of('k');
            if (kPos != std::string::npos)
            {
                std::string kNumStr;
                for (size_t i = kPos + 1; i < nameStr.size(); ++i)
                {
                    if (isdigit(nameStr[i]))
                        kNumStr += nameStr[i];
                    else
                        break;
                }
                if (!kNumStr.empty())
                    vehicleCount = std::stoi(kNumStr);
            }
        }
        else if (key == "TYPE")
        {
            std::string colon, typeStr;
            iss >> colon >> typeStr;
            if (typeStr != "CVRP")
            {
                throw std::runtime_error("仅支持CVRP问题类型");
            }
        }
        else if (key == "DIMENSION")
        { // 注意原文件中的拼写错误应为"DIMENSION"
            std::string colon;
            iss >> colon >> dimension;
        }
        else if (key == "CAPACITY")
        {
            std::string colon;
            iss >> colon >> capacity;
        }
        else if (key == "EDGE_WEIGHT_TYPE")
        {
            std::string colon, ewt;
            iss >> colon >> ewt;
            if (ewt != "EUC_2D")
            {
                throw std::runtime_error("仅支持EUC_2D距离计算");
            }
        }
        else if (key == "NODE_COORD_SECTION")
        {
            nodes.reserve(dimension);
            for (int i = 0; i < dimension; ++i)
            {
                std::string coordLine;
                std::getline(file, coordLine);
                std::istringstream coordIss(coordLine);
                int id, x, y;
                if (!(coordIss >> id >> x >> y))
                {
                    throw std::runtime_error("节点坐标解析错误");
                }
                nodes.emplace_back(id, x, y, 0);
                idToIndex[id] = nodes.size() - 1;
            }
        }
        else if (key == "DEMAND_SECTION")
        {
            for (int i = 0; i < dimension; ++i)
            {
                std::string demandLine;
                std::getline(file, demandLine);
                std::istringstream demandIss(demandLine);
                int id, demand;
                if (!(demandIss >> id >> demand))
                {
                    throw std::runtime_error("需求解析错误");
                }
                auto it = idToIndex.find(id);
                if (it == idToIndex.end())
                {
                    throw std::runtime_error("未知节点ID: " + std::to_string(id));
                }
                nodes[it->second].demand = demand;
            }
        }
        else if (key == "DEPOT_SECTION")
        {
            // 读取仓库节点
            while (true)
            {
                std::string depotLine;
                std::getline(file, depotLine);
                std::istringstream depotIss(depotLine);
                int id;
                while (depotIss >> id)
                {
                    if (id == -1)
                        break;
                    if (depotId == -1)
                        depotId = id; // 仅取第一个仓库
                }
                if (id == -1)
                    break;
            }
        }
        else if (key == "COMMENT")
        {
            // COMMENT 后面可能有多个字段，直接整行匹配
            std::smatch match;
            std::regex pattern(R"(Optimal value:\s*(\d+))");
            if (std::regex_search(line, match, pattern))
            {
                optimalV = std::stoi(match[1].str());
                // std::cout << "Optimal value: " << optimalValue << std::endl;
            }
        }
        else if (key == "EOF")
        {
            break;
        }
    }

    // 找到仓库节点在 nodes 中的 index
    auto depotIt = idToIndex.find(depotId);
    if (depotIt == idToIndex.end())
    {
        throw std::runtime_error("仓库节点未找到");
    }
    size_t depotIndex = depotIt->second;

    // 创建新的节点数组，id范围调整为 0 ~ n-1，仓库放 index 0
    std::vector<Node> newNodes;
    newNodes.reserve(nodes.size());

    // 先添加 depot 节点，并设置 id 为 0
    Node depotNode = nodes[depotIndex];
    depotNode.id = 0;
    newNodes.push_back(depotNode);

    // 添加其他节点（跳过 depotId）
    for (const auto &node : nodes)
    {
        if (node.id == depotId)
            continue;

        Node newNode = node;
        if (node.id > depotId)
        {
            newNode.id = node.id - 1;
        }
        // 如果 node.id < depotId 则 id 保持不变
        newNodes.push_back(newNode);
    }

    // 重建 idToIndex
    idToIndex.clear();
    for (size_t i = 0; i < newNodes.size(); ++i)
    {
        idToIndex[newNodes[i].id] = i;
    }

    // 更新 model
    Model model = Model(vehicleCount, capacity);
    model.nodes = std::move(newNodes);
    model.computeDistances();
    model.optimalVal = optimalV;

    // for (auto n : model.nodes)
    //     printf("%d %lf %lf\n", n.id, n.x, n.y);

    return model;
}
