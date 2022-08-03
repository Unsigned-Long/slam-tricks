#ifndef PEOJECTIVE_H
#define PEOJECTIVE_H

#include "artwork/logger/logger.h"
#include "eigen3/Eigen/Dense"
#include "opencv2/core.hpp"

namespace ns_st11 {

  Eigen::Matrix3d solveHomoMat(const std::vector<cv::Point2d> &pc1,
                               const std::vector<cv::Point2d> &pc2);

  Eigen::Matrix3d solveHomoMatByRANSAC(const std::vector<cv::Point2d> &pc1,
                                       const std::vector<cv::Point2d> &pc2,
                                       float errorThd,
                                       std::size_t iter = 20);

  cv::Point2d projectHomoMat(const cv::Point2d &p, const Eigen::Matrix3d &hMat);

} // namespace st_11

#endif