//
// Created by csl on 11/4/22.
//

#include "lidar.h"

int main(int argc, char **argv) {
    ns_st19::LidarScene LidarScene("/home/csl/CppWorks/artwork/slam-tricks/st19-distortion/img");
    LidarScene.Run();
    return 0;
}