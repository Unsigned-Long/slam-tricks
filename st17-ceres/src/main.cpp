//
// Created by csl on 10/18/22.
//

#ifndef ST17_CERES_MAIN_H
#define ST17_CERES_MAIN_H

#include "scene.h"
#include "solver.hpp"

using namespace ns_st17;


std::pair<Posed, Posed> CameraPose() {
    Posed real, init;
    {
        constexpr float yaw = -120.0, pitch = 110.0, roll = 0.0;
        auto y = Eigen::AngleAxisd(DegreeToRadian(yaw), Eigen::Vector3d(0.0, 0.0, 1.0));
        auto p = Eigen::AngleAxisd(DegreeToRadian(pitch), Eigen::Vector3d(1.0, 0.0, 0.0));
        auto r = Eigen::AngleAxisd(DegreeToRadian(roll), Eigen::Vector3d(0.0, 1.0, 0.0));
        // world to camera
        auto angleAxis = r * p * y;
        real = Posed::fromRt(angleAxis.inverse().matrix(), Eigen::Vector3d(3, 2, 1));
    }
    {
        constexpr float yaw = -90.0, pitch = 100.0, roll = 10.0;
        auto y = Eigen::AngleAxisd(DegreeToRadian(yaw), Eigen::Vector3d(0.0, 0.0, 1.0));
        auto p = Eigen::AngleAxisd(DegreeToRadian(pitch), Eigen::Vector3d(1.0, 0.0, 0.0));
        auto r = Eigen::AngleAxisd(DegreeToRadian(roll), Eigen::Vector3d(0.0, 1.0, 0.0));
        // world to camera
        auto angleAxis = r * p * y;
        init = Posed::fromRt(angleAxis.inverse().matrix(), Eigen::Vector3d(3.5, 0.0, 0.0));
    }
    return {real, init};
}

std::vector<CubePlane> CubePlanes() {
    return {
            CubePlane(0.0f, 0.0f, 0.0f, -5.0f, 0.0f, 0.0f, 10.0f),
            CubePlane(0.0f, 0.0f, 90.0f, 0.0f, +5.0f, 0.0f, 10.0f),
            CubePlane(0.0f, 0.0f, 0.0f, +5.0f, 0.0f, 0.0f, 10.0f),
            CubePlane(0.0f, 0.0f, 90.0f, 0.0f, -5.0f, 0.0f, 10.0f),

            CubePlane(90.0f, 0.0f, 0.0f, 0.0f, 0.0f, -2.25f, 10.0f, 10.0f)
    };
}

int main(int argc, char **argv) {
    try {

        Scene scene(Colour(1.0, 1.0, 1.0), true);

        auto cubePlanes = CubePlanes();
        auto [CtoW_REAL, CtoW_INIT] = CameraPose();

        // cube plane sequence
        // important param:
        // [0] lineMode: Whether to use line drawing
        // [1]  opacity: transparency
        scene.AddCubes(cubePlanes, false, 0.2f);

        scene.AddCamera("CtoW_REAL", CtoW_REAL, 0.0, 1.0, 0.0);
        scene.AddCamera("CtoW_INIT", CtoW_INIT, 0.0, 0.0, 0.0);

        auto WtoC = Posed::fromSE3(CtoW_REAL.se3().inverse());
        std::vector<CorrPair> corrs;
        std::size_t idx = 0;
        for (const auto &cubePlane: cubePlanes) {
            for (const auto &p: (*cubePlane._features)) {
                auto pInW = Eigen::Vector3d(p.x, p.y, p.z);
                auto pInC = WtoC.se3() * pInW;
                const auto X = pInC(0), Y = pInC(1), Z = pInC(2);
                if (Z > 0.0) {
                    const auto normX = X / Z, normY = Y / Z;
                    if (normX > -1.0 && normX < 1.0 && normY > -0.75 && normY < 0.75) {
                        corrs.emplace_back(pInW, Eigen::Vector2d(normX, normY));
                        // draw
                        scene.AddLine(
                                pcl::PointXYZ(pInW.x(), pInW.y(), pInW.z()),
                                pcl::PointXYZ(CtoW_REAL.t(0), CtoW_REAL.t(1), CtoW_REAL.t(2)),
                                "Line-" + std::to_string(idx)
                        );
                        ++idx;
                    }
                }
            }
        }

        LOG_INFO("the valid feature num: ", idx)
        LOG_INFO("the real pose: ", CtoW_REAL)
        LOG_INFO("the init pose: ", CtoW_INIT)

        {
            auto result = SolvePnPWithDynamicAutoDiff(corrs, CtoW_INIT.so3, CtoW_INIT.t);
            auto pose = Posed::fromSE3(result);
            LOG_INFO("the opt pose: ", pose)
        }
        // ---
        // run
        // ---
        scene.RunSingleThread();

    } catch (const std::exception &e) {
        LOG_ERROR(e.what());
    }
    return 0;
}

#endif //ST17_CERES_MAIN_H
