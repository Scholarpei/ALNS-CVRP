// 程序入口

// parameter
const static int MAXFEs = 5e4;

#include "../include/Model.h"
#include "../include/ALNS.h"

void test_for_loading_model()
{
    Model model = Model::loadFromFile("A-n32-k5.vrp");
    model.computeDistances();
    model.print();
}

int main()
{
    Model model = Model::loadFromFile("data/..?..");
    ALNS alns(model);

    Solution bestSolution = alns.runALNS();

    return 0;
}
