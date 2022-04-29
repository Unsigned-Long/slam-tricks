#ifndef RECOVERY_H
#define RECOVERY_H

#include "eigen3/Eigen/Dense"
#include "opencv2/core.hpp"
#include "struct_def.hpp"
#include "triangulation.hpp"
#include <iostream>

using namespace ns_st0;

namespace ns_st2 {
  /**
   * @brief recovery the movement from the essential matrix
   *
   * @param eMatrix the essential matrix
   * @param K the camera's inner paramters
   * @param kp1 the key point in first frame
   * @param kp2 the key point in second frame
   * @param rot21 rotation matrix from first frame to second frame
   * @param t21 translation matrix from first frame to second frame
   * @return true the process is successful
   * @return false the process is failed
   */
  static bool recoveryMove(
      const Eigen::Matrix3f &eMatrix,
      const Eigen::Matrix3f &K,
      const cv::KeyPoint &kp1,
      const cv::KeyPoint &kp2,
      Eigen::Matrix3f &rot21,
      Eigen::Vector3f &t21) {

    // SVD decomposition
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(eMatrix.normalized(), Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector3f singularVal = svd.singularValues();
    Eigen::Matrix3f uMatrix = svd.matrixU();
    Eigen::Matrix3f vMatrix = svd.matrixV();

    // normalize sigma matrix
    float sigma = 0.5f * (singularVal(0) + singularVal(1));
    Eigen::Matrix3f sigmaMatrix = Eigen::Matrix3f::Zero();
    sigmaMatrix(0, 0) = sigmaMatrix(1, 1) = sigma;

    // temp matrices
    Eigen::Matrix3f pRotMat = Eigen::Matrix3f::Zero();
    pRotMat(0, 1) = -1.0f, pRotMat(1, 0) = 1.0, pRotMat(2, 2) = 1.0f;

    Eigen::Matrix3f nRotMat = Eigen::Matrix3f::Zero();
    nRotMat(0, 1) = 1.0f, nRotMat(1, 0) = -1.0, nRotMat(2, 2) = 1.0f;

    // to normalize a rotation matrix
    auto normRot = [](Eigen::Matrix3f &rot) -> void {
      // for loop two times
      for (int i = 0; i != 2; ++i) {
        // normalize rows
        Eigen::Vector3f row1 = rot.row(0);
        Eigen::Vector3f row2 = rot.row(1).normalized();
        Eigen::Vector3f row3 = row1.cross(row2).normalized();
        row1 = row2.cross(row3);
        rot.row(0) = row1;
        rot.row(1) = row2;
        rot.row(2) = row3;

        // normalize cols
        Eigen::Vector3f col1 = rot.col(0);
        Eigen::Vector3f col2 = rot.col(1).normalized();
        Eigen::Vector3f col3 = col1.cross(col2).normalized();
        col1 = col2.cross(col3);
        rot.col(0) = col1;
        rot.col(1) = col2;
        rot.col(2) = col3;
      }
    };

    // check a solution is right
    auto checkSolution = [&kp1, &kp2, &K, &rot21, &t21](const Eigen::Matrix3f &rot, const Eigen::Vector3f &t) -> bool {
      // compute the depth
      std::pair<float, float> depth = triangulation(kp1, kp2, K, rot, t);
      
      // if two values are positive
      if (depth.first > 0.0f && depth.second > 0.0f) {
        rot21 = rot;
        t21 = t;
        return true;
      } else {
        return false;
      }
    };

    // different solutions
    // solution 1
    Eigen::Matrix3f R1 = uMatrix * pRotMat.transpose() * vMatrix.transpose();
    normRot(R1);
    Eigen::Vector3f t1 = ns_st0::antisymmetric(Eigen::Matrix3f(uMatrix * pRotMat * sigmaMatrix * uMatrix.transpose())).normalized();
    if (checkSolution(R1, t1)) {
      return true;
    }

    // solution 2
    Eigen::Matrix3f R2 = R1;
    Eigen::Vector3f t2 = -t1;
    if (checkSolution(R2, t2)) {
      return true;
    }

    // solution 3
    Eigen::Matrix3f R3 = uMatrix * nRotMat.transpose() * vMatrix.transpose();
    normRot(R3);
    Eigen::Vector3f t3 = ns_st0::antisymmetric(Eigen::Matrix3f(uMatrix * nRotMat * sigmaMatrix * uMatrix.transpose())).normalized();
    if (checkSolution(R3, t3)) {
      return true;
    }

    // solution 4
    Eigen::Matrix3f R4 = R3;
    Eigen::Vector3f t4 = -t3;
    if (checkSolution(R4, t4)) {
      return true;
    }

    return false;
  }
} // namespace ns_st2

#endif