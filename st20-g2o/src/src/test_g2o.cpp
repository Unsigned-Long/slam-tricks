//
// Created by csl on 11/5/22.
//

#include "test_g2o.h"

int main(int argc, char **argv) {
    ns_st20::ProblemScene problemScene(100, "/home/csl/CppWorks/artwork/slam-tricks/st20-g2o/scene");
    // problemScene.ShowCameraAt(10);
    // problemScene.ShowFeatureAt(10);
    // problemScene.ShowCameras();
    // problemScene.ShowFeatures();
    auto simData = problemScene.Simulation(true, 0.3f, 3.0f);
    // auto img = simData.Hessian();
    // cv::imwrite("/home/csl/CppWorks/artwork/slam-tricks/st20-g2o/scene/hessian-100.png", img);
    simData.Run("/home/csl/CppWorks/artwork/slam-tricks/st20-g2o/scene", "Simulation Data");
    ns_st20::SolveWithG2O(simData, true);
    return 0;
}
