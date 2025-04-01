// CVRP 模型
#include "Model.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <algorithm>

using namespace std;

Model::Model(int vehicleCount, int capacity)
    : vehicleCount(vehicleCount), capacity(capacity) {}

void Model::computeDistances()
{
    const double self2self = 1e9;  // 定义节点自身距离

    distanceMatrix.clear();
    for (const auto& from : nodes) {
        for (const auto& to : nodes) {
            // 如果是自身节点，设为INF
            if (from.id == to.id) {
                distanceMatrix[from.id][to.id] = self2self;
            }
            else {
                // 正常计算欧式距离
                double dx = from.x - to.x;
                double dy = from.y - to.y;
                distanceMatrix[from.id][to.id] = std::sqrt(dx * dx + dy * dy);
            }
        }
    }
}

void Model::print() const {
    // 基本信息
    std::cout << "\n========== Model Status ==========\n";
    std::cout << "Vehicles: " << vehicleCount
        << " | Capacity: " << capacity
        << " | Nodes: " << nodes.size() << "\n\n";

    // 节点列表（按ID排序）
    std::vector<Node> sortedNodes = nodes;
    std::sort(sortedNodes.begin(), sortedNodes.end(),
        [](const Node& a, const Node& b) { return a.id < b.id; });

    std::cout << "=== Node List (ID | Coordinates | Demand) ===\n";
    std::cout << std::left
        << std::setw(6) << "ID"
        << std::setw(15) << "Coordinates"
        << "Demand\n";

    for (const auto& node : sortedNodes) {
        std::cout << std::setw(6) << node.id
            << "(" << std::setw(4) << node.x
            << "," << std::setw(4) << node.y << ")"
            << std::setw(10) << ""
            << std::setw(4) << node.demand << "\n";
    }

    // 距离矩阵样本
    std::cout << "\n=== Distance Matrix Sample ===\n";
    if (distanceMatrix.empty()) {
        std::cout << "Distance matrix not computed!\n";
        return;
    }

    // 打印前5个节点间的距离
    const int sampleSize = std::min(5, static_cast<int>(nodes.size()));
    std::cout << std::fixed << std::setprecision(2);

    // 表头
    std::cout << std::setw(8) << "From\\To";
    for (int j = 0; j < sampleSize; ++j) {
        std::cout << std::setw(8) << sortedNodes[j].id;
    }
    std::cout << "\n";

    // 表格内容
    for (int i = 0; i < sampleSize; ++i) {
        std::cout << std::setw(8) << sortedNodes[i].id;
        for (int j = 0; j < sampleSize; ++j) {
            auto from = sortedNodes[i].id;
            auto to = sortedNodes[j].id;
            std::cout << std::setw(8) << distanceMatrix.at(from).at(to);
        }
        std::cout << "\n";
    }

    // 仓库节点验证
    auto depot = std::find_if(nodes.begin(), nodes.end(),
        [](const Node& n) { return n.demand == 0; });
    if (depot != nodes.end()) {
        std::cout << "\n[Depot] ID: " << depot->id
            << " | Location: (" << depot->x << ", " << depot->y << ")\n";
    }
}

void Model::initialSolution()
{
    // to be written
}

Model Model::loadFromFile(const string& filename)
{
    std::ifstream file("../data/" + filename);
    if (!file.is_open()) {
        throw std::runtime_error("无法打开文件: " + filename);
    }

    std::string line;
    std::string name, type, edgeWeightType;
    int dimension = 0, capacity = 0, vehicleCount = 0;
    std::vector<Node> nodes;
    std::unordered_map<int, size_t> idToIndex;
    int depotId = -1;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string key;
        if (!(iss >> key)) continue;

        if (key == "NAME") {
            std::string colon, nameStr;
            iss >> colon >> nameStr;
            // 从名称中提取车辆数量，例如"A-n32-k5"中的k5
            size_t kPos = nameStr.find_last_of('k');
            if (kPos != std::string::npos) {
                std::string kNumStr;
                for (size_t i = kPos + 1; i < nameStr.size(); ++i) {
                    if (isdigit(nameStr[i])) kNumStr += nameStr[i];
                    else break;
                }
                if (!kNumStr.empty()) vehicleCount = std::stoi(kNumStr);
            }
        }
        else if (key == "TYPE") {
            std::string colon, typeStr;
            iss >> colon >> typeStr;
            if (typeStr != "CVRP") {
                throw std::runtime_error("仅支持CVRP问题类型");
            }
        }
        else if (key == "DIMENSION") { // 注意原文件中的拼写错误应为"DIMENSION"
            std::string colon;
            iss >> colon >> dimension;
        }
        else if (key == "CAPACITY") {
            std::string colon;
            iss >> colon >> capacity;
        }
        else if (key == "EDGE_WEIGHT_TYPE") {
            std::string colon, ewt;
            iss >> colon >> ewt;
            if (ewt != "EUC_2D") {
                throw std::runtime_error("仅支持EUC_2D距离计算");
            }
        }
        else if (key == "NODE_COORD_SECTION") {
            nodes.reserve(dimension);
            for (int i = 0; i < dimension; ++i) {
                std::string coordLine;
                std::getline(file, coordLine);
                std::istringstream coordIss(coordLine);
                int id, x, y;
                if (!(coordIss >> id >> x >> y)) {
                    throw std::runtime_error("节点坐标解析错误");
                }
                nodes.emplace_back(id, x, y, 0);
                idToIndex[id] = nodes.size() - 1;
            }
        }
        else if (key == "DEMAND_SECTION") {
            for (int i = 0; i < dimension; ++i) {
                std::string demandLine;
                std::getline(file, demandLine);
                std::istringstream demandIss(demandLine);
                int id, demand;
                if (!(demandIss >> id >> demand)) {
                    throw std::runtime_error("需求解析错误");
                }
                auto it = idToIndex.find(id);
                if (it == idToIndex.end()) {
                    throw std::runtime_error("未知节点ID: " + std::to_string(id));
                }
                nodes[it->second].demand = demand;
            }
        }
        else if (key == "DEPOT_SECTION") {
            // 读取仓库节点
            while (true) {
                std::string depotLine;
                std::getline(file, depotLine);
                std::istringstream depotIss(depotLine);
                int id;
                while (depotIss >> id) {
                    if (id == -1) break;
                    if (depotId == -1) depotId = id; // 仅取第一个仓库
                }
                if (id == -1) break;
            }
        }
        else if (key == "EOF") {
            break;
        }
    }

    // 验证仓库节点
    auto depotIt = idToIndex.find(depotId);
    if (depotIt == idToIndex.end()) {
        throw std::runtime_error("仓库节点未找到");
    }
    if (nodes[depotIt->second].demand != 0) {
        throw std::runtime_error("仓库节点需求不为零");
    }

    Model model(vehicleCount, capacity);
    model.nodes = nodes;
    model.computeDistances();

    return model;
}
