#include <iostream>
#include <filesystem>
#include <vector>
#include <string>
#include <numeric>
#include <iomanip>
#include <random>
#include <fstream> //  用于写入CSV

#include "../include/Model.h"
#include "../include/ALNS.h"
#include "../include/options.h"

namespace fs = std::filesystem;

const int NUM_RUNS = 25;
const std::string DATA_DIR = "../data/";
// const std::string DATA_DIR = "D:/project/ALNS-CVRP/dataonlyone";

void run()
{
    std::cout << "调试开始" << std::endl; // 设置断点
    std::vector<double> allInstanceAvgErrors;

    fs::create_directory("output"); // 创建输出文件夹（如果不存在）

    for (const auto &entry : fs::directory_iterator(DATA_DIR))
    {
        if (entry.is_regular_file() && entry.path().extension() == ".vrp")
        {
            std::string filepath = entry.path().string();
            std::cout << "Running instance: " << filepath << std::endl;

            std::vector<double> errors;

            for (int i = 0; i < NUM_RUNS; ++i)
            {
                Model model = Model::loadFromFile(DATA_DIR, filepath);
                model.computeDistances();
                ALNS alns(model, std::random_device{}());

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

            // 写入到 CSV 文件
            std::string filename = entry.path().stem().string(); // 不含扩展名
            std::ofstream outFile("output/" + filename + ".csv");
            outFile << "Run,Error\n";
            for (size_t i = 0; i < errors.size(); ++i)
            {
                outFile << (i + 1) << "," << std::fixed << std::setprecision(6) << errors[i] << "\n";
            }
            double avgError = std::accumulate(errors.begin(), errors.end(), 0.0) / errors.size();
            outFile << "Average," << std::fixed << std::setprecision(6) << avgError << "\n";
            outFile.close();

            allInstanceAvgErrors.push_back(avgError);

            std::cout << "Average error for " << filepath << ": "
                      << std::fixed << std::setprecision(2) << avgError << " %" << std::endl;
        }
    }

    double totalAvgError = std::accumulate(allInstanceAvgErrors.begin(), allInstanceAvgErrors.end(), 0.0) / allInstanceAvgErrors.size();
    std::cout << "=====================================" << std::endl;
    std::cout << "Overall average error across all instances: "
              << std::fixed << std::setprecision(2) << totalAvgError << " %" << std::endl;
}

int main()
{
    try
    {
        run();
    }
    catch (const std::exception &e)
    {
        std::cerr << "[ERROR] Exception caught in main: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "[ERROR] Unknown exception caught in main." << std::endl;
    }

    std::cout << "[Main] Finished" << std::endl;

    // Model model = Model::loadFromFile("../data/A-n32-k5.vrp");
    // ALNS alns(model, SEED);

    // Solution bestSolution = alns.runALNS(
    //     model.logger,
    //     RAND_D_MIN,
    //     RAND_D_MAX,
    //     WORST_D_MIN,
    //     WORST_D_MAX,
    //     REGRET_N,
    //     R1,
    //     R2,
    //     R3,
    //     PHI,
    //     RHO);

    // printf("Finish Test!\n");
    return 0;
}
