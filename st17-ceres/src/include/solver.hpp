//
// Created by csl on 10/18/22.
//

#ifndef PNP_SOLVER_HPP
#define PNP_SOLVER_HPP

#include <utility>

#include "Eigen/Dense"

namespace ns_st17 {
    struct CorrPair {
        Eigen::Vector3d point;
        Eigen::Vector2d feature;

        CorrPair(Eigen::Vector3d point, Eigen::Vector2d feature)
                : point(std::move(point)), feature(std::move(feature)) {}

        CorrPair() = default;
    };
}

#endif //PNP_SOLVER_HPP
