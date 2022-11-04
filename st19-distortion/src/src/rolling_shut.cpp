//
// Created by csl on 11/4/22.
//

#include "rolling_shut.h"

int main(int argc, char **argv) {
    ns_st19::CameraScene cameraScene("/home/csl/CppWorks/artwork/slam-tricks/st19-distortion/img");
    cameraScene.SetShutSpeedInv(2);
    cameraScene.Run();
    return 0;
}