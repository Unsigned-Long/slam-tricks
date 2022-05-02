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
   * @return Eigen::Matrix3d the function matrix
   */
  static Eigen::Matrix3f solveEpipolar(
      const std::vector<cv::KeyPoint> &kps1,
      const std::vector<cv::KeyPoint> &kps2,
      const std::vector<cv::DMatch> &srcMatches,
      const CameraInnerParam &innerParam,
      std::vector<cv::DMatch> *goodMatches = nullptr,
      double quantile = 1.323, double scaleFactor = 1.2, uint layerNum = 8) {

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

    // key point on different layer with different sigma
    std::vector<double> sigma2(layerNum);
    sigma2.front() = 1.0f;
    double layerScale = 1.0;
    for (int i = 1; i != layerNum; ++i) {
      layerScale *= scaleFactor;
      sigma2.at(i) = layerScale * layerScale;
    }

    // normalize
    std::vector<Eigen::Vector2d> normKps1(matches.size()), normKps2(matches.size());
    Eigen::Vector2d pm1 = Eigen::Vector2d::Zero(), pm2 = Eigen::Vector2d::Zero();
    Eigen::Vector2d pd1 = Eigen::Vector2d::Zero(), pd2 = Eigen::Vector2d::Zero();

    // compute the mean
    for (int i = 0; i != matches.size(); ++i) {
      const auto &match = matches.at(i);
      const cv::KeyPoint &kp1 = kps1.at(match.queryIdx);
      const cv::KeyPoint &kp2 = kps2.at(match.trainIdx);

      normKps1.at(i) = Eigen::Vector2d(kp1.pt.x, kp1.pt.y);
      normKps2.at(i) = Eigen::Vector2d(kp2.pt.x, kp2.pt.y);

      pm1 += normKps1.at(i);
      pm2 += normKps2.at(i);
    }
    std::cout << std::endl;
    pm1 /= matches.size(), pm2 /= matches.size();

    // compute the norm variance
    for (int i = 0; i != matches.size(); ++i) {

      normKps1.at(i) -= pm1;
      normKps2.at(i) -= pm2;

      const Eigen::Vector2d &nkp1 = normKps1.at(i);
      const Eigen::Vector2d &nkp2 = normKps2.at(i);

      pd1 += Eigen::Vector2d(std::abs(nkp1(0)), std::abs(nkp1(1)));
      pd2 += Eigen::Vector2d(std::abs(nkp2(0)), std::abs(nkp2(1)));
    }
    pd1 /= matches.size(), pd2 /= matches.size();

    double us1 = 1.0 / pd1(0), vs1 = 1.0 / pd1(1);
    double us2 = 1.0 / pd2(0), vs2 = 1.0 / pd2(1);
    double um1 = pm1(0), vm1 = pm1(1);
    double um2 = pm2(0), vm2 = pm2(1);

    // normalize variance
    for (int i = 0; i != matches.size(); ++i) {
      Eigen::Vector2d &nkp1 = normKps1.at(i);
      Eigen::Vector2d &nkp2 = normKps2.at(i);

      nkp1(0) *= us1, nkp1(1) *= vs1;
      nkp2(0) *= us2, nkp2(1) *= vs2;
    }

    Eigen::Matrix3d N1 = Eigen::Matrix3d::Identity(), N2 = Eigen::Matrix3d::Identity();
    N1(0, 0) = us1, N1(0, 2) = -um1 * us1;
    N1(1, 1) = vs1, N1(1, 2) = -vm1 * vs1;
    N2(0, 0) = us2, N2(0, 2) = -um2 * us2;
    N2(1, 1) = vs2, N2(1, 2) = -vm2 * vs2;

    // matrices for least square
    Eigen::MatrixXd matA(matches.size(), 9);
    Eigen::Matrix3d matF;

    // record the outliers' index in the matches
    std::set<int> outliers;

    // construct the A matrix
    for (int i = 0; i != matches.size(); ++i) {
      const Eigen::Vector2d &nkp1 = normKps1.at(i);
      const Eigen::Vector2d &nkp2 = normKps2.at(i);

      double nu1 = nkp1(0), nv1 = nkp1(1);
      double nu2 = nkp2(0), nv2 = nkp2(1);

      matA(i, 0) = nu1 * nu2, matA(i, 1) = nv1 * nu2, matA(i, 2) = nu2;
      matA(i, 3) = nu1 * nv2, matA(i, 4) = nv1 * nv2, matA(i, 5) = nv2;
      matA(i, 6) = nu1, matA(i, 7) = nv1, matA(i, 8) = 1.0;
    }

    // find outliers [the condition is to ensure that the equation has a solution]
    while (matches.size() - outliers.size() > 8) {

      // solve
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(matA, Eigen::ComputeFullV);
      Eigen::MatrixXd vMatrix = svd.matrixV();
      matF = vMatrix.col(8).reshaped(3, 3).transpose();
      matF = N2.transpose() * matF * N1;

      // find the badest outliers
      double maxVar = 0.0;
      int maxIdx = -1;

      for (int i = 0; i != matches.size(); ++i) {
        // if current match was a invaild match, then continue
        if (outliers.count(i) != 0) {
          continue;
        }

        const cv::DMatch &match = matches.at(i);
        const cv::KeyPoint &kp1 = kps1.at(match.queryIdx);
        const cv::KeyPoint &kp2 = kps2.at(match.trainIdx);

        double u1 = kp1.pt.x, v1 = kp1.pt.y;
        double u2 = kp2.pt.x, v2 = kp2.pt.y;

        Eigen::Vector3d p1(u1, v1, 1.0);
        Eigen::Vector3d p2(u2, v2, 1.0);
        Eigen::Matrix<double, 3, 1> temp1 = matF * p1;
        Eigen::Matrix<double, 1, 3> temp2 = p2.transpose() * matF;

        double a1 = temp1(0, 0), b1 = temp1(1, 0), c1 = temp1(2, 0);
        double a2 = temp2(0, 0), b2 = temp2(0, 1), c2 = temp2(0, 2);

        double num1 = a1 * u2 + b1 * v2 + c1;
        double den1 = a1 * a1 + b1 * b1;

        double num2 = a2 * u1 + b2 * v1 + c2;
        double den2 = a2 * a2 + b2 * b2;

        if (den1 == 0.0f || den2 == 0.0f) {
          continue;
        }

        // reproject to frame 2
        double statistics1 = num1 * num1 / den1;
        // reproject to frame 1
        double statistics2 = num2 * num2 / den2;

        if (statistics1 < quantile * sigma2.at(kp2.octave) &&
            statistics2 < quantile * sigma2.at(kp1.octave)) {
          // current match is a good match
          continue;
        }

        double msta = (statistics1 + statistics2) * 0.5f;

        if (maxVar < msta) {
          maxVar = msta;
          maxIdx = i;
        }
      }

      if (maxIdx == -1) {
        // which means no outlier in current left matches
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

    Eigen::Matrix3f matE;
    Eigen::Matrix3f K = innerParam.toEigenMatrix<float>();
    matE = K.transpose() * matF.cast<float>() * K;
    matE.normalize();

    return matE;
  }
} // namespace ns_st2

#endif