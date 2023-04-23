//
// Created by csl on 4/23/23.
//

#include "two_view_geometry.h"
#include "artwork/logger/logger.h"

namespace ns_st22 {

    std::optional<Sophus::SE3d>
    TwoViewGeometry::FindFunctionalMatrix(const std::vector<FeaturePair> &pairs, const Eigen::Matrix3d &kMat) {
        Eigen::Matrix3d fMat = ComputeFunctionMatrix(pairs);
        LOG_PROCESS("the function matrix: \n", fMat)
        auto SE3_F2toF1 = DecomposeFMat(fMat, kMat, pairs);
        return SE3_F2toF1;
    }

    Eigen::Matrix3d TwoViewGeometry::ComputeFunctionMatrix(const std::vector<FeaturePair> &pairs) {
        Eigen::MatrixXd AMat(pairs.size(), 9);
        for (int i = 0; i < pairs.size(); ++i) {
            const auto &p = pairs.at(i);
            auto u1 = p.f1(0), v1 = p.f1(1);
            auto u2 = p.f2(0), v2 = p.f2(1);
            AMat(i, 0) = u1 * u2;
            AMat(i, 1) = u1 * v2;
            AMat(i, 2) = u1;
            AMat(i, 3) = v1 * u2;
            AMat(i, 4) = v1 * v2;
            AMat(i, 5) = v1;
            AMat(i, 6) = u2;
            AMat(i, 7) = v2;
            AMat(i, 8) = 1.0;
        }
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(AMat, Eigen::ComputeFullV | Eigen::ComputeFullU);
        const Eigen::MatrixXd &vMat = svd.matrixV();

        Eigen::Matrix3d fMat = Eigen::Matrix3d(vMat.col(8).data()).transpose();

        return fMat;
    }

    std::optional<Sophus::SE3d>
    TwoViewGeometry::DecomposeFMat(const Eigen::Matrix3d &fMat, const Eigen::Matrix3d &kMat,
                                   const std::vector<FeaturePair> &pairs) {
        Eigen::Matrix3d eMat = kMat.transpose() * fMat * kMat;

        Eigen::JacobiSVD<Eigen::Matrix3d> svd(eMat, Eigen::ComputeFullV | Eigen::ComputeFullU);
        const Eigen::Matrix3d &uMat = svd.matrixU();
        const Eigen::Matrix3d &vMat = svd.matrixV();
        Eigen::Matrix3d sigmaMat = Eigen::Matrix3d::Zero();
        sigmaMat(0, 1) = -1.0;
        sigmaMat(1, 0) = 1.0;
        sigmaMat(2, 2) = 1.0;

        Eigen::Vector3d tVec1 = uMat.col(2), tVec2 = -tVec1;
        Eigen::Matrix3d rMat1 = uMat * sigmaMat * vMat.transpose();
        Eigen::Matrix3d rMat2 = uMat * sigmaMat.transpose() * vMat.transpose();

        if (rMat1.determinant() < 0.0) { rMat1 = -rMat1; }
        if (rMat2.determinant() < 0.0) { rMat2 = -rMat2; }

        auto b1 = CheckRotMatTransVec(rMat1, tVec1, kMat, pairs);
        auto b2 = CheckRotMatTransVec(rMat1, tVec2, kMat, pairs);
        auto b3 = CheckRotMatTransVec(rMat2, tVec1, kMat, pairs);
        auto b4 = CheckRotMatTransVec(rMat2, tVec2, kMat, pairs);
        if (std::multiset<bool>{b1, b2, b3, b4}.count(true) == 1) {
            // successfully
            if (b1) {
                return Sophus::SE3d{AdjustRotationMatrix(rMat1), tVec1};
            } else if (b2) {
                return Sophus::SE3d{AdjustRotationMatrix(rMat1), tVec2};
            } else if (b3) {
                return Sophus::SE3d{AdjustRotationMatrix(rMat2), tVec1};
            } else if (b4) {
                return Sophus::SE3d{AdjustRotationMatrix(rMat2), tVec2};
            }
        } else {
            // failed
            return {};
        }
    }

    bool TwoViewGeometry::CheckRotMatTransVec(const Eigen::Matrix3d &rMat, const Eigen::Vector3d &tVec,
                                              const Eigen::Matrix3d &kMat, const std::vector<FeaturePair> &pairs) {
        for (const auto &p: pairs) {
            // [frame 1 to frame 1 | frame 2 to frame 1], get landmark in frame 1
            Eigen::Vector3d pInF1 = Triangulate(
                    p, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), rMat, tVec, kMat
            );
            // invalid
            if (pInF1(2) <= 0.0) {
                return false;
            }
            Eigen::Vector3d pInF2 = rMat.transpose() * pInF1 - rMat.transpose() * tVec;
            // invalid
            if (pInF2(2) <= 0.0) {
                return false;
            }
        }
        return true;
    }

    Eigen::Vector3d
    TwoViewGeometry::Triangulate(const FeaturePair &p, const Eigen::Matrix3d &rMat1, const Eigen::Vector3d &tVec1,
                                 const Eigen::Matrix3d &rMat2, const Eigen::Vector3d &tVec2,
                                 const Eigen::Matrix3d &kMat) {
        Eigen::Matrix<double, 3, 4> P1;
        P1.block<3, 3>(0, 0) = rMat1.transpose();
        P1.block<3, 1>(0, 3) = -rMat1.transpose() * tVec1;
        P1 = kMat * P1;

        Eigen::Matrix<double, 3, 4> P2;
        P2.block<3, 3>(0, 0) = rMat2.transpose();
        P2.block<3, 1>(0, 3) = -rMat2.transpose() * tVec2;
        P2 = kMat * P2;

        Eigen::Matrix<double, 6, 4> AMat;
        AMat.block<3, 4>(0, 0) = Sophus::SO3d::hat(Eigen::Vector3d(p.f1(0), p.f1(1), 1.0)) * P1;
        AMat.block<3, 4>(3, 0) = Sophus::SO3d::hat(Eigen::Vector3d(p.f2(0), p.f2(1), 1.0)) * P2;

        Eigen::JacobiSVD<Eigen::Matrix<double, 6, 4>> svd(AMat, Eigen::ComputeFullV | Eigen::ComputeFullU);
        const Eigen::Matrix4d &vMat = svd.matrixV();
        Eigen::Vector4d lm = vMat.col(3);

        return Eigen::Vector4d(lm / lm(3)).block<3, 1>(0, 0);
    }
}
