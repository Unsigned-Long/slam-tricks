//
// Created by csl on 10/16/22.
//
#include "scene.h"

int main(int argc, char **argv) {
    try {
        using namespace ns_st16;
        ns_log::ns_priv::stdCoutLogger.setPrecision(9);

        LOG_INFO("adjust the parameters and press 'Alter' key to save the current scene.")
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
        // important param:
        // [0] background: the background color of this scene
        // [1] addOriginCoord : Whether to add the reference coordinate system
        Scene scene(Colour(1.0, 1.0, 1.0), true);

        // pose sequence
        // scene.AddPoseSeq("odomIMU", odomIMU, 10, 0.3f);
        // scene.AddPoseSeq("odomLiDAR", odomLiDAR);
        // important param:
        // [0] downSample: If the coordinate system identities is too dense, use this parameter
        // [1] size: the size(or scale) of the coordinate system identity
        scene.AddPoseSeq("odomVicon", odomVicon, 2, 0.3f);

        // cube plane sequence
        // important param:
        // [0] lineMode: Whether to use line drawing
        // [1]  opacity: transparency
        scene.AddCubes(
                {
                        CubePlane(0.0f, 0.0f, 0.0f, 0.0f, -3.0f, 0.0f, 1.4f),
                        CubePlane(0.0f, 0.0f, 90.0f, 0.7f, -3.7f, 0.0f, 1.4f),

                        CubePlane(0.0f, 0.0f, 0.0f, 0.0f - 2.0f, -3.0f, 0.0f, 1.4f),
                        CubePlane(0.0f, 0.0f, 90.0f, 0.7f - 2.0f, -3.7f, 0.0f, 1.4f),

                        CubePlane(0.0f, 0.0f, 0.0f, 0.0f - 4.0f, -3.0f, 0.0f, 1.4f),
                        CubePlane(0.0f, 0.0f, 90.0f, 0.7f - 4.0f, -3.7f, 0.0f, 1.4f),

                        CubePlane(0.0f, 0.0f, 0.0f, 0.0f - 6.0f, -3.0f, 0.0f, 1.4f),
                        CubePlane(0.0f, 0.0f, 90.0f, 0.7f - 6.0f, -3.7f, 0.0f, 1.4f),

                        CubePlane(0.0f, 0.0f, 45.0f, -6.0f, 0.0f, 0.0f, 2.5f),

                        CubePlane(0.0f, 0.0f, 0.0f, -6.0f, 2.5f, 0.0f, 1.6f),
                        CubePlane(0.0f, 0.0f, 0.0f, -6.0f, 4.0f, 0.0f, 0.8f),
                        CubePlane(0.0f, 0.0f, 90.0f, -6.0f + 0.4f, 4.0f + 0.4f, 0.0f, 0.8f),

                        CubePlane(0.0f, 0.0f, 0.0f, -4.0f, 2.5f, 0.0f, 1.6f),
                        CubePlane(0.0f, 0.0f, 0.0f, -4.0f, 4.0f, 0.0f, 0.8f),
                        CubePlane(0.0f, 0.0f, 90.0f, -4.0f + 0.4f, 4.0f + 0.4f, 0.0f, 0.8f),

                        CubePlane(0.0f, 0.0f, 60.0f, 0.0f, 4.0f, 0.0f, 3.0f),

                        CubePlane(0.0f, 0.0f, 0.0f, 2.0f, 0.0f + 0.5f, 0.0f, 10.0f),
                        CubePlane(0.0f, 0.0f, 90.0f, -3.0f, 5.0f + 0.5f, 0.0f, 10.0f),
                        CubePlane(0.0f, 0.0f, 0.0f, -8.0f, 0.0f + 0.5f, 0.0f, 10.0f),
                        CubePlane(0.0f, 0.0f, 90.0f, -3.0f, -5.0f + 0.5f, 0.0f, 10.0f),

                        CubePlane(90.0f, 0.0f, 0.0f, -3.0f, 0.5f, -2.25f, 10.0f, 10.0f),
                        CubePlane(90.0f, 0.0f, 0.0f, -3.0f, 0.5f, 2.25f, 10.0f, 10.0f)
                }, true, 1.0f
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