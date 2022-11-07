//
// Created by csl on 11/5/22.
//

#include "test_ceres.h"

int main(int argc, char **argv) {
    ns_st20::ProblemScene problemScene(100, "/home/csl/CppWorks/artwork/slam-tricks/st20-g2o/scene");
    // problemScene.ShowCameraAt(10);
    // problemScene.ShowFeatureAt(10);
    // problemScene.ShowCameras();
    // problemScene.ShowFeatures();
    auto simData = problemScene.Simulation(0.3f, 30.0f, "/home/csl/CppWorks/artwork/slam-tricks/st20-g2o/scene");
    simData.DrawScene();
    ns_st20::SolveWithDynamicAutoDiff(simData, false);
    return 0;
}
