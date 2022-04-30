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
   * @param scaleFactor the scale factor used in orb feature algorithm
   * @param layerNum the layer total quantity in orb feature algorithm
   * @return Eigen::Matrix3f the function matrix
   */
  static Eigen::Matrix3f solveEpipolar(
      const std::vector<cv::KeyPoint> &kps1,
      const std::vector<cv::KeyPoint> &kps2,
      const std::vector<cv::DMatch> &srcMatches,
      const CameraInnerParam &innerParam,
      std::vector<cv::DMatch> *goodMatches = nullptr,
      float quantile = 1.323, float scaleFactor = 1.2f, uint layerNum = 8) {

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

    CV_Assert(matches.size() >= 8);

    // matrices for least square
    Eigen::MatrixXf matA(matches.size(), 8), vecl = -Eigen::VectorXf::Ones(matches.size());
    Eigen::Vector<float, 8> vecX;

    // record the outliers' index in the matches
    std::set<int> outliers;

    float fx = innerParam.fx, fy = innerParam.fy, fxInv = 1.0f / fx, fyInv = 1.0f / fy;
    float cx = innerParam.cx, cy = innerParam.cy;

    // key point on different layer with different sigma
    std::vector<float> sigma2(layerNum);
    sigma2.front() = 1.0f;
    float layerScale = 1.0;
    for (int i = 1; i != layerNum; ++i) {
      layerScale *= scaleFactor;
      sigma2.at(i) = layerScale * layerScale;
    }

    Eigen::Matrix3f K = innerParam.toEigenMatrix<float>(), KInv = K.inverse();
    Eigen::Matrix3f matF, matE;

    // construct the A matrix
    for (int i = 0; i != matches.size(); ++i) {
      const auto &match = matches.at(i);
      const cv::KeyPoint &kp1 = kps1.at(match.queryIdx);
      const cv::KeyPoint &kp2 = kps2.at(match.trainIdx);

      float u1 = kp1.pt.x, v1 = kp1.pt.y;
      float u2 = kp2.pt.x, v2 = kp2.pt.y;

      float x1 = (u1 - cx) * fxInv, y1 = (v1 - cy) * fyInv;
      float x2 = (u2 - cx) * fxInv, y2 = (v2 - cy) * fyInv;

      matA(i, 0) = x1 * x2, matA(i, 1) = y1 * x2;
      matA(i, 2) = x2, matA(i, 3) = x1 * y2;
      matA(i, 4) = y1 * y2, matA(i, 5) = y2;
      matA(i, 6) = x1, matA(i, 7) = y1;
    }

    // find outliers [the condition is to ensure that the equation has a solution]
    while (matches.size() - outliers.size() > 8) {

      // solve
      vecX = (matA.transpose() * matA).inverse() * matA.transpose() * vecl;

      matE(0, 0) = vecX(0), matE(0, 1) = vecX(1), matE(0, 2) = vecX(2);
      matE(1, 0) = vecX(3), matE(1, 1) = vecX(4), matE(1, 2) = vecX(5);
      matE(2, 0) = vecX(6), matE(2, 1) = vecX(7), matE(2, 2) = 1.0f;

      matE.normalize();

      matF = KInv.transpose() * matE * KInv;

      // find the badest outliers
      float maxVar = 0.0;
      int maxIdx = -1;

      for (int i = 0; i != matches.size(); ++i) {
        // if current match was a invaild match, then continue
        if (outliers.count(i) != 0) {
          continue;
        }

        const auto &match = matches.at(i);
        const cv::KeyPoint &kp1 = kps1.at(match.queryIdx);
        const cv::KeyPoint &kp2 = kps2.at(match.trainIdx);

        float u1 = kp1.pt.x, v1 = kp1.pt.y;
        float u2 = kp2.pt.x, v2 = kp2.pt.y;

        Eigen::Vector3f p2(u2, v2, 1.0f);
        Eigen::Matrix<float, 1, 3> temp = p2.transpose() * matF;

        float a = temp(0, 0), b = temp(0, 1), c = temp(0, 2);

        float num = a * u1 + b * v1 + c;
        float den = a * a + b * b;

        if (den == 0.0) {
          continue;
        }

        float statistics = num * num / den;

        if (statistics < quantile * sigma2.at(kps2.at(match.trainIdx).octave)) {
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

    return matE;
  }
} // namespace ns_st2

#endif