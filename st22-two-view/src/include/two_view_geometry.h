//
// Created by csl on 4/23/23.
//

#ifndef TWO_VIEW_GEOMETRY_TWO_VIEW_GEOMETRY_H
#define TWO_VIEW_GEOMETRY_TWO_VIEW_GEOMETRY_H

#include "two_view_simu.h"

namespace ns_st22 {
    struct TwoViewGeometry {
    public:
        static std::optional<Sophus::SE3d>
        FindFunctionalMatrix(const std::vector<FeaturePair> &pairs, const Eigen::Matrix3d &kMat);

    protected:
        static Eigen::Matrix3d ComputeFunctionMatrix(const std::vector<FeaturePair> &pairs);

        static std::optional<Sophus::SE3d>
        DecomposeFMat(const Eigen::Matrix3d &fMat, const Eigen::Matrix3d &kMat, const std::vector<FeaturePair> &pairs);

        static bool CheckRotMatTransVec(const Eigen::Matrix3d &rMat, const Eigen::Vector3d &tVec,
                                        const Eigen::Matrix3d &kMat, const std::vector<FeaturePair> &pairs);

    public:
        static Eigen::Vector3d
        Triangulate(const FeaturePair &p, const Eigen::Matrix3d &rMat1, const Eigen::Vector3d &tVec1,
                    const Eigen::Matrix3d &rMat2, const Eigen::Vector3d &tVec2, const Eigen::Matrix3d &kMat);
    };
}


#endif //TWO_VIEW_GEOMETRY_TWO_VIEW_GEOMETRY_H
