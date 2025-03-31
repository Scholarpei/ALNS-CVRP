// 程序入口

// parameter
const static int MAXFEs = 5e4;

#include "Model.h"
#include "ALNS.h"

int main()
{
    Model model = Model::loadFromFile("data/..?..");
    ALNS alns(model);

    Solution bestSolution = alns.runALNS(MAXFEs);

    return 0;
}
