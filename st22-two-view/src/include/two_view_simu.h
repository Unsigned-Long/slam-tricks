//
// Created by csl on 4/23/23.
//

#ifndef ST22_TWO_VIEW_TWO_VIEW_SIMU_H
#define ST22_TWO_VIEW_TWO_VIEW_SIMU_H

#include <utility>
#include "eigen3/Eigen/Dense"
#include "random"
#include "optional"
#include "chrono"
#include "sophus/se3.hpp"
#include "slam-scene-viewer/scene_viewer.h"

namespace ns_st22 {

    template<class ScalarType>
    inline Sophus::Matrix3<ScalarType> AdjustRotationMatrix(const Sophus::Matrix3<ScalarType> &rotMat) {
        // adjust
        Eigen::JacobiSVD<Sophus::Matrix3<ScalarType>> svd(rotMat, Eigen::ComputeFullV | Eigen::ComputeFullU);
        const Sophus::Matrix3<ScalarType> &vMatrix = svd.matrixV();
        const Sophus::Matrix3<ScalarType> &uMatrix = svd.matrixU();
        Sophus::Matrix3<ScalarType> adjustedRotMat = uMatrix * vMatrix.transpose();
        return adjustedRotMat;
    }

    struct FeaturePair {
    public:
        /**
         * @brief the members
         */
        Eigen::Vector2d f1;
        Eigen::Vector2d f2;
        Eigen::Vector3d pInW;
        Eigen::Vector3d pInF1;
        Eigen::Vector3d pInF2;

    public:
        /**
         * @brief construct a new FeaturePair object
         */
        FeaturePair(Eigen::Vector2d f1, Eigen::Vector2d f2, Eigen::Vector3d pInW, Eigen::Vector3d pInF1,
                    Eigen::Vector3d pInF2)
                : f1(std::move(f1)), f2(std::move(f2)), pInW(std::move(pInW)), pInF1(std::move(pInF1)),
                  pInF2(std::move(pInF2)) {}

        /**
         * @brief override operator '<<' for type 'FeaturePair'
         */
        friend std::ostream &operator<<(std::ostream &os, const FeaturePair &obj) {
            os << '{';
            os << "'f1': " << obj.f1.transpose() << ", 'f2': " << obj.f2.transpose()
               << ", 'pInW': " << obj.pInW.transpose() << ", 'pInF1': " << obj.pInF1.transpose() << ", 'pInF2': "
               << obj.pInF2.transpose();
            os << '}';
            return os;
        }
    };


    std::optional<Eigen::Vector2d>
    WorldLandmarkToImage(const Sophus::SE3d &SE3_FtoW, const Eigen::Matrix3d &kMat, const Eigen::Vector3d &lm,
                         int width, int height);

    std::pair<std::vector<FeaturePair>, Sophus::SE3d>
    SimulateTwoViewObservations(const Eigen::Matrix3d &kMat, int width, int height, int lmSize = 100,
                                bool displayScene = true);

}

#endif //ST22_TWO_VIEW_TWO_VIEW_SIMU_H
