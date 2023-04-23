//
// Created by csl on 4/23/23.
//
#include "two_view_simu.h"
#include "two_view_geometry.h"
#include "artwork/logger/logger.h"

int main() {
    // ----------
    // simulation
    // ----------
    double fx = 400.0, fy = 400.0, cx = 300.0, cy = 200.0;
    int width = 600, height = 400;
    Eigen::Matrix3d kMat;
    kMat << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0;
    auto [pairs, SE3_F2toF1] = ns_st22::SimulateTwoViewObservations(kMat, width, height);
    LOG_INFO("SE3 from frame 2 to frame 1: \n", SE3_F2toF1.matrix3x4())

    // --------
    // recovery
    // --------
    if (auto pose = ns_st22::TwoViewGeometry::FindFunctionalMatrix(pairs, kMat);pose) {
        LOG_INFO("the found SE3 from frame 2 to frame 1 from functional matrix: \n", pose->matrix3x4())
        double scale = SE3_F2toF1.translation().norm() / pose->translation().norm();
        auto scaledPose = Sophus::SE3d(pose->so3(), pose->translation() * scale);
        LOG_INFO("scaled: \n", scaledPose.matrix3x4())

        // triangulation
        for (const auto &p: pairs) {
            Eigen::Vector3d pInF1 = ns_st22::TwoViewGeometry::Triangulate(
                    p, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(),
                    scaledPose.rotationMatrix(), scaledPose.translation(), kMat
            );
            LOG_VAR(pInF1.transpose(), p.pInF1.transpose())
        }
    } else {
        LOG_ERROR("find SE3 from frame 2 to frame 1 from functional matrix failed.")
    }
    return 0;
}