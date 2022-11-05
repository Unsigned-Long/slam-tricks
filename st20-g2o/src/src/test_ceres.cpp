//
// Created by csl on 11/5/22.
//

#include "test_ceres.h"
#include "sim_data.h"

int main(int argc, char **argv) {
    ns_st20::ProblemScene problemScene;
    problemScene.Run(10);
    LOG_INFO("Hello, world!")
    return 0;
}
