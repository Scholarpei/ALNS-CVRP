// CVRP 模型
#include "Model.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <limits>

using namespace std;

Model::Model(int vehicleCount, int capacity)
    : vehicleCount(vehicleCount), capacity(capacity) {}

void Model::computeDistances()
{
    // to be written
}

void Model::initialSolution()
{
    // to be written
}

double Model::evaluateSolution(const Solution &sol) const
{
    double totalCost = 0.0;

    // to be written

    return totalCost;
}

Model Model::loadFromFile(const string &filename)
{
    // to be written

    // model.computeDistances();
    // model.initialSolution();

    return model;
}
