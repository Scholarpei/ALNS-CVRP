// 程序入口

// parameter
const static int MAXFEs = 5e4;

#include "../include/Model.h"
#include "../include/ALNS.h"
#include "../include/options.h"

void test_for_loading_model()
{
    Model model = Model::loadFromFile("A-n32-k5.vrp");
    model.computeDistances();
    model.print();
}

int main()
{
    Model model = Model::loadFromFile("../data/A-n32-k5.vrp");
    ALNS alns(model);

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
        PHI);

    printf("Finish Test!\n");

    return 0;
}
