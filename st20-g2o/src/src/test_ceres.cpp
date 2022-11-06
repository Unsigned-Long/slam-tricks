//
// Created by csl on 11/5/22.
//

#include "test_ceres.h"
#include "sim_data.h"

int main(int argc, char **argv) {
    ns_st20::ProblemScene problemScene("/home/csl/CppWorks/artwork/slam-tricks/st20-g2o/scene");
//     problemScene.ShowCameraAt(10);
//     problemScene.ShowFeatureAt(10);
//    problemScene.ShowCameras();
    problemScene.ShowFeatures();
    LOG_INFO("Hello, world!")
    return 0;
}
