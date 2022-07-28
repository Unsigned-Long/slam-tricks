#ifndef HELP_H
#define HELP_H

#define FORMAT_VECTOR
#include "artwork/logger/logger.h"
#include "cbcorners.h"
#include "eigen3/Eigen/Dense"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace ns_st3 {
  /**
   * @brief a function to get all the filenames in the directory
   * @param directory the file directory
   * @return the filenames in the directory
   */
  std::vector<std::string> filesInDir(const std::string &directory);

  double homoError(const CBPtsVec &imgPts, const CBPtsVec &objPts, const Eigen::Matrix3d &hMat);

  Eigen::Vector3d toHomoCoordVec(const cv::Point2d &pt);

} // namespace ns_st3

#endif