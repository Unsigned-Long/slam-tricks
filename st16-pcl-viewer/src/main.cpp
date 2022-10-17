//
// Created by csl on 10/16/22.
//
#include "scene.h"

int main(int argc, char **argv) {
    try {
        using namespace ns_st16;
        ns_log::ns_priv::stdCoutLogger.setPrecision(9);
        // ---------
        // read data
        // ---------
        // the imu and lidar is not expressed in the same coordinate, and we don't know the bias, so just use the 'odom_vicon'
        // auto odomIMU = ns_st16::Scene::ReadOdom("../data/odom_imu/odometryInfo.txt");
        // auto odomLiDAR = ns_st16::Scene::ReadOdom("../data/odom_lidar/odometryInfo.txt");
        auto odomVicon = Scene::ReadOdom("../data/odom_vicon/odometryInfo.txt");
        // ----
        // show
        // ----
        Scene scene(Colour(1.0, 1.0, 1.0), true);

        // pose sequence
        // scene.AddPoseSeq("odomIMU", odomIMU, 10, 0.3f);
        // scene.AddPoseSeq("odomLiDAR", odomLiDAR);
        scene.AddPoseSeq("odomVicon", odomVicon, 2, 0.3f);

        // cube plane sequence
        scene.AddCubes(
                {
                        CubePlane(Posed(), 1.0, 1.0, 1.0)
                }, false, 0.5f
        );

        // ---
        // run
        // ---
        scene.RunSingleThread();

    } catch (const std::exception &e) {
        LOG_ERROR(e.what());
    }
    return 0;
}