#ifndef EPIPOLAR_H
#define EPIPOLAR_H

#include "eigen3/Eigen/Dense"
#include "opencv2/core.hpp"
#include "struct_def.hpp"
#include <algorithm>
#include <iostream>
#include <set>

using namespace ns_st0;

namespace ns_st2 {
  /**
   * @brief to get function matrix based on the epipolar constraints
   *
   * @param kps1 the keypoints in the first image
   * @param kps2 the keypoints in the second image
   * @param srcMatches the source matches, It can be matching data without preprocessing
   * @param goodMatches the good matches that this algorithm return
   * @param quantile the quantile to judge whether a match is an outlier
   * @return Eigen::Matrix3f the function matrix
   */
  static Eigen::Matrix3f solveEpipolar(
      const std::vector<cv::KeyPoint> &kps1,
      const std::vector<cv::KeyPoint> &kps2,
      const std::vector<cv::DMatch> &srcMatches,
      std::vector<cv::DMatch> *goodMatches = nullptr,
      float quantile = 3.8415) {

    CV_Assert(srcMatches.size() >= 8);

    // clean source data
    std::vector<cv::DMatch> matches;
    matches.reserve(0.5 * srcMatches.size());

    auto minDisIter = std::min_element(
        srcMatches.cbegin(), srcMatches.cend(),
        [](const cv::DMatch &m1, const cv::DMatch &m2) {
          return m1.distance < m2.distance;
        });

    for (int i = 0; i != srcMatches.size(); ++i) {
      // filter bad matches
      if (srcMatches.at(i).distance < std::max(30.0f, 2.0f * minDisIter->distance)) {
        matches.push_back(srcMatches.at(i));
      }
    }

    // matrices for least square
    Eigen::MatrixXd matA(matches.size(), 8), vecl = -Eigen::VectorXd::Ones(matches.size());
    Eigen::Vector<double, 8> matX;

    // record the outliers' index in the matches
    std::set<int> outliers;

    // construct the A matrix and l vector
    for (int i = 0; i != matches.size(); ++i) {
      const auto &match = matches.at(i);

      float u1 = kps1.at(match.queryIdx).pt.x, v1 = kps1.at(match.queryIdx).pt.y;
      float u2 = kps2.at(match.trainIdx).pt.x, v2 = kps2.at(match.trainIdx).pt.y;

      matA(i, 0) = u1 * u2, matA(i, 1) = v1 * u2;
      matA(i, 2) = u2, matA(i, 3) = u1 * v2;
      matA(i, 4) = v1 * v2, matA(i, 5) = v2;
      matA(i, 6) = u1, matA(i, 7) = v1;
    }

    // find outliers [the condition is to ensure that the equation has a solution]
    while (matches.size() - outliers.size() > 8) {

      // solve
      matX = (matA.transpose() * matA).inverse() * matA.transpose() * vecl;

      // get parameters
      double f1 = matX(0, 0), f2 = matX(1, 0), f3 = matX(2, 0), f4 = matX(3, 0);
      double f5 = matX(4, 0), f6 = matX(5, 0), f7 = matX(6, 0), f8 = matX(7, 0);

      // find the badest outliers
      double maxVar = 0.0;
      int maxIdx = -1;

      for (int i = 0; i != matches.size(); ++i) {
        // if current match was a invaild match, then continue
        if (outliers.count(i) != 0) {
          continue;
        }

        const auto &match = matches.at(i);
        float u1 = kps1.at(match.queryIdx).pt.x, v1 = kps1.at(match.queryIdx).pt.y;
        float u2 = kps2.at(match.trainIdx).pt.x, v2 = kps2.at(match.trainIdx).pt.y;

        double a = u2 * f1 + v2 * f4 + f7;
        double b = u2 * f2 + v2 * f5 + f8;
        double c = u2 * f3 + v2 * f6 + 1.0;

        double num = a * u1 + b * v1 + c;
        double den = a * a + b * b;

        if (den == 0.0) {
          continue;
        }

        double statistics = num * num / den;

        if (statistics < quantile) {
          // current match is a good match
          continue;
        }

        if (maxVar < statistics) {
          maxVar = statistics;
          maxIdx = i;
        }
      }

      if (maxIdx == -1) {
        // which means no outliers in current left matches
        break;
      } else {
        // remove the outlier's affect
        matA.row(maxIdx).setZero();
        outliers.insert(maxIdx);
      }
    }

    // organize F matrix
    Eigen::Matrix3f matF;

    matF(0, 0) = matX(0, 0), matF(0, 1) = matX(1, 0), matF(0, 2) = matX(2, 0);
    matF(1, 0) = matX(3, 0), matF(1, 1) = matX(4, 0), matF(1, 2) = matX(5, 0);
    matF(2, 0) = matX(6, 0), matF(2, 1) = matX(7, 0), matF(2, 2) = 1.0f;

    if (goodMatches != nullptr) {
      goodMatches->clear();
      goodMatches->resize(matches.size() - outliers.size());
      int count = 0;
      for (int i = 0; i != matches.size(); ++i) {
        if (outliers.count(i) == 0) {
          // it's not a outliers
          goodMatches->at(count++) = (matches.at(i));
        }
      }
    }

    return matF;
  }
} // namespace ns_st2

#endif