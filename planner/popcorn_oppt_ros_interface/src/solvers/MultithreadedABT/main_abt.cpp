#include "oppt/problemEnvironment/ProblemEnvironment.hpp"
#include "solverABTMultithreaded.hpp"
#include "MultithreadedABTOptions.hpp"


int main(int argc, char const* argv[])
{    
    oppt::ProblemEnvironment problemEnvironment;
    int ret = problemEnvironment.setup<solvers::ABTMultithreaded, solvers::MultithreadedABTOptions>(argc, argv);
    if (ret != 0)
        return ret;
    return problemEnvironment.runEnvironment(argc, argv);
}
