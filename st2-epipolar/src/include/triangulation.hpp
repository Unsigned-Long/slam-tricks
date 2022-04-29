#ifndef TRIANGULATION_H
#define TRIANGULATION_H

#include "eigen3/Eigen/Dense"
#include "opencv2/core.hpp"
#include "struct_def.hpp"
#include <iostream>

namespace ns_st2 {
  /**
   * @brief to trangular a pair points on the normalized coordinate
   *
   * @param X1 the point on the first camera's normalized coordinate
   * @param X2 the point on the second camera's normalized coordinate
   * @param rot21 the rotation from first camera to second camera
   * @param t21 the translation from first camera to second camera
   * @param P1 the point on the first camera's coordinate
   * @param P2 the point on the second camera's coordinate
   * @return std::pair<float, float> the depth pair
   */
  static std::pair<float, float> triangulation(
      const Eigen::Vector3f &X1,
      const Eigen::Vector3f &X2,
      const Eigen::Matrix3f &rot21,
      const Eigen::Vector3f &t21,
      Eigen::Vector3f *P1 = nullptr,
      Eigen::Vector3f *P2 = nullptr) {

    // the depth
    float s1, s2;

    // to solve s1
    Eigen::Vector3f aVec = ns_st0::antisymmetric(X2) * rot21 * X1;
    Eigen::Vector3f lVec = -ns_st0::antisymmetric(X2) * t21;
    s1 = ((aVec.transpose() * aVec).inverse() * aVec.transpose() * lVec)(0, 0);

    // to solve s2
    Eigen::Vector3f bVec = X2;
    Eigen::Vector3f mVec = s1 * rot21 * X1 + t21;
    s2 = ((bVec.transpose() * bVec).inverse() * bVec.transpose() * mVec)(0, 0);

    if (P1 != nullptr) {
      *P1 = s1 * X1;
    }

    if (P2 != nullptr) {
      *P2 = s2 * X2;
    }

    return {s1, s2};
  }

  /**
   * @brief to trangular a pair points on the pixel coordinate
   *
   * @param P1 the point on the first camera's pixel coordinate
   * @param P2 the point on the second camera's pixel coordinate
   * @param K the camera's inner parameter matrix
   * @param rot21 the rotation from first camera to second camera
   * @param t21 the translation from first camera to second camera
   * @param P1 the point on the first camera's coordinate
   * @param P2 the point on the second camera's coordinate
   * @return std::pair<float, float> the depth pair
   */
  static std::pair<float, float> triangulation(
      const cv::KeyPoint &p1,
      const cv::KeyPoint &p2,
      const Eigen::Matrix3f &K,
      const Eigen::Matrix3f &rot21,
      const Eigen::Vector3f &t21,
      Eigen::Vector3f *P1 = nullptr,
      Eigen::Vector3f *P2 = nullptr) {
    Eigen::Vector3f X1 = K.inverse() * Eigen::Vector3f(p1.pt.x, p1.pt.y, 1.0f);
    Eigen::Vector3f X2 = K.inverse() * Eigen::Vector3f(p2.pt.x, p2.pt.y, 1.0f);
    return triangulation(X1, X2, rot21, t21, P1, P2);
  }
} // namespace ns_st2

#endif