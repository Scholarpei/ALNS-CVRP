#include <iostream>
#include <filesystem>
#include <vector>
#include <string>
#include <numeric> // for std::accumulate
#include <iomanip> // for std::setprecision
#include <random>

#include "../include/Model.h"
#include "../include/ALNS.h"
#include "../include/options.h"

namespace fs = std::filesystem;

const int NUM_RUNS = 25;
const std::string DATA_DIR = "../data/";

void run()
{
    std::vector<double> allInstanceAvgErrors; // 存储每个实例的平均误差

    for (const auto &entry : fs::directory_iterator(DATA_DIR))
    {
        if (entry.is_regular_file() && entry.path().extension() == ".vrp")
        {
            std::string filepath = entry.path().string();
            std::cout << "Running instance: " << filepath << std::endl;

            std::vector<double> errors;

            for (int i = 0; i < NUM_RUNS; ++i)
            {
                Model model = Model::loadFromFile(filepath);
                model.computeDistances();
                ALNS alns(model, std::random_device{}()); // 每次用不同种子可以增加多样性

                Solution bestSolution = alns.runALNS(
                    model.logger,
                    RAND_D_MIN,
                    RAND_D_MAX,
                    WORST_D_MIN,
                    WORST_D_MAX,
                    REGRET_N,
                    R1,
                    R2,
                    R3,
                    PHI,
                    RHO);

                double error = ((bestSolution.total_distance - model.optimalVal) / model.optimalVal) * 100.0;
                errors.push_back(error);
            }

            double avgError = std::accumulate(errors.begin(), errors.end(), 0.0) / errors.size();
            allInstanceAvgErrors.push_back(avgError);

            std::cout << "Average error for " << filepath << ": "
                      << std::fixed << std::setprecision(2) << avgError << " %" << std::endl;
        }
    }

    // 计算所有实例的总体平均误差
    double totalAvgError = std::accumulate(allInstanceAvgErrors.begin(), allInstanceAvgErrors.end(), 0.0) / allInstanceAvgErrors.size();
    std::cout << "=====================================" << std::endl;
    std::cout << "Overall average error across all instances: "
              << std::fixed << std::setprecision(2) << totalAvgError << " %" << std::endl;
}

int main()
{
    run();
    return 0;
}
