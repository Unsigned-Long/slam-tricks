#ifndef AFFINE_HPP
#define AFFINE_HPP

#include "artwork/geometry/point.hpp"
#include "artwork/logger/logger.h"
#include "eigen3/Eigen/Dense"
#include <functional>

namespace ns_st9 {

  static Eigen::Matrix3d projective(const ns_geo::PointSet2d &pc1, const ns_geo::PointSet2d &pc2) {
    Eigen::Vector<double, 8> param = Eigen::Vector<double, 8>::Zero();
    int count = pc1.size();
    for (int i = 0; i != 10; ++i) {
      Eigen::Matrix<double, 8, 8> HMat = Eigen::Matrix<double, 8, 8>::Zero();
      Eigen::Vector<double, 8> gMat = Eigen::Vector<double, 8>::Zero();
      for (int j = 0; j != count; ++j) {
        const auto &p1 = pc1[j], p2 = pc2[j];
        double x1 = p1.x, y1 = p1.y;
        double x2 = p2.x, y2 = p2.y;
        double a = param(0), b = param(1), c = param(2);
        double d = param(3), e = param(4), f = param(5);
        double g = param(6), h = param(7);
        // compute the jacobian matrix and error vector
        Eigen::Matrix<double, 8, 2> J = Eigen::Matrix<double, 8, 2>::Zero();
        // X: a, b
        J(0, 0) = x1, J(1, 0) = y1, J(2, 0) = 1.0;
        J(3, 1) = x1, J(4, 1) = y1, J(5, 1) = 1.0;
        J(6, 0) = -x1 * x2, J(7, 0) = -x2 * y1;
        J(6, 1) = -x1 * y2, J(7, 1) = -y2 * y1;
        Eigen::Vector2d eVec;
        eVec(0) = a * x1 + b * y1 + c - x1 * x2 * g - x2 * y1 * h - x2;
        eVec(1) = d * x1 + e * y1 + f - x1 * y2 * g - y2 * y1 * h - y2;
        // update
        HMat += J * J.transpose();
        gMat -= (J * eVec);
      }
      // solve
      Eigen::Vector<double, 8> delta = HMat.ldlt().solve(gMat);
      LOG_VAR(i, delta.norm());
      param += delta;
      if (delta.norm() < 1E-10) {
        break;
      }
    }
    double a = param(0), b = param(1), c = param(2);
    double d = param(3), e = param(4), f = param(5);
    double g = param(6), h = param(7);
    Eigen::Matrix3d trans;
    trans(0, 0) = a, trans(0, 1) = b, trans(0, 2) = c;
    trans(1, 0) = d, trans(1, 1) = e, trans(1, 2) = f;
    trans(2, 0) = g, trans(2, 1) = h, trans(2, 2) = 1.0;
    return trans;
  }

  static std::function<ns_geo::Point2d(const ns_geo::Point2d &p)>
  projectiveForward(const ns_geo::PointSet2d &pc1, const ns_geo::PointSet2d &pc2) {
    Eigen::Matrix3d trans = projective(pc1, pc2);
    // organize the function
    return [trans](const ns_geo::Point2d &p) -> ns_geo::Point2d {
      Eigen::Vector3d vp(p.x, p.y, 1.0);
      Eigen::Vector3d rp = trans * vp;
      rp /= rp(2);
      return ns_geo::Point2d(rp(0), rp(1));
    };
  }

  static std::function<ns_geo::Point2d(const ns_geo::Point2d &p)>
  projectiveBackward(const ns_geo::PointSet2d &pc1, const ns_geo::PointSet2d &pc2) {
    Eigen::Matrix3d trans = projective(pc1, pc2).inverse();
    // organize the function
    return [trans](const ns_geo::Point2d &p) -> ns_geo::Point2d {
      Eigen::Vector3d vp(p.x, p.y, 1.0);
      Eigen::Vector3d rp = trans * vp;
      rp /= rp(2);
      return ns_geo::Point2d(rp(0), rp(1));
    };
  }

  namespace svd {
    static Eigen::Matrix3d projective_svd(const ns_geo::PointSet2d &pc1, const ns_geo::PointSet2d &pc2) {
      const ushort count = pc1.size();
      Eigen::MatrixXd A(count * 2, 9);
      for (int i = 0; i != count; ++i) {
        const auto &p1 = pc1[i], p2 = pc2[i];
        double x1 = p1.x, y1 = p1.y;
        double x2 = p2.x, y2 = p2.y;
        // row 0
        A(i * 2, 0) = x1, A(i * 2, 1) = y1, A(i * 2, 2) = 1.0;
        A(i * 2, 3) = A(i * 2, 4) = A(i * 2, 5) = 0.0;
        A(i * 2, 6) = -x1 * x2, A(i * 2, 7) = -x2 * y1, A(i * 2, 8) = -x2;
        // row 1
        A(i * 2 + 1, 0) = A(i * 2 + 1, 1) = A(i * 2 + 1, 2) = 0.0;
        A(i * 2 + 1, 3) = x1, A(i * 2 + 1, 4) = y1, A(i * 2 + 1, 5) = 1.0;
        A(i * 2 + 1, 6) = -x1 * y2, A(i * 2 + 1, 7) = -y2 * y1, A(i * 2 + 1, 8) = -y2;
      }
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
      Eigen::Matrix<double, 9, 9> vMatrix = svd.matrixV();
      Eigen::Vector<double, 9> param = vMatrix.col(8);
      // construct
      double a = param(0), b = param(1), c = param(2);
      double d = param(3), e = param(4), f = param(5);
      double g = param(6), h = param(7), i = param(8);
      Eigen::Matrix3d trans;
      trans(0, 0) = a, trans(0, 1) = b, trans(0, 2) = c;
      trans(1, 0) = d, trans(1, 1) = e, trans(1, 2) = f;
      trans(2, 0) = g, trans(2, 1) = h, trans(2, 2) = i;
      return trans;
    }

    static std::function<ns_geo::Point2d(const ns_geo::Point2d &p)>
    projectiveForward_svd(const ns_geo::PointSet2d &pc1, const ns_geo::PointSet2d &pc2) {
      Eigen::Matrix3d trans = projective_svd(pc1, pc2);
      // organize the function
      return [trans](const ns_geo::Point2d &p) -> ns_geo::Point2d {
        Eigen::Vector3d vp(p.x, p.y, 1.0);
        Eigen::Vector3d rp = trans * vp;
        rp /= rp(2);
        return ns_geo::Point2d(rp(0), rp(1));
      };
    }

    static std::function<ns_geo::Point2d(const ns_geo::Point2d &p)>
    projectiveBackward_svd(const ns_geo::PointSet2d &pc1, const ns_geo::PointSet2d &pc2) {
      Eigen::Matrix3d trans = projective_svd(pc1, pc2).inverse();
      // organize the function
      return [trans](const ns_geo::Point2d &p) -> ns_geo::Point2d {
        Eigen::Vector3d vp(p.x, p.y, 1.0);
        Eigen::Vector3d rp = trans * vp;
        rp /= rp(2);
        return ns_geo::Point2d(rp(0), rp(1));
      };
    }
  } // namespace svd

} // namespace ns_st9

#endif