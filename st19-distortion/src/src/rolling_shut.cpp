//
// Created by csl on 11/4/22.
//

#include "rolling_shut.h"

int main(int argc, char **argv) {
    ns_st19::CameraScene cameraScene;
    cameraScene.RunInMultiThread();
    std::cin.get();
    cameraScene.SetFinished();
    return 0;
}