#ifndef ICP_H
#define ICP_H

#include "artwork/geometry/point.hpp"
#include "artwork/logger/logger.h"
#include "sophus/se2.hpp"

namespace ns_st6 {
  using point_cloud_2d = std::vector<Eigen::Vector2f>;
  static std::pair<point_cloud_2d, point_cloud_2d> generatePts(const Sophus::SE2f &T21, uint count) {
    auto ps1 = ns_geo::PointSet2f::randomGenerator(count, 0.0f, 10.0f, 0.0f, 10.0f);
    auto error = ns_geo::PointSet2f::randomGenerator(count, -0.1f, 0.1f, -0.1f, 0.1f);
    std::vector<Eigen::Vector2f> pc1, pc2;
    // generate point cloud 1
    for (const auto &elem : ps1) {
      pc1.push_back(Eigen::Vector2f(elem.x, elem.y));
    }
    // generate point cloud 2
    for (int i = 0; i != count; ++i) {
      auto e = error.at(i);
      auto p = Eigen::Vector2f(e.x, e.y) + T21 * pc1.at(i);
      pc2.push_back(p);
    }
    return {pc1, pc2};
  }

  static Sophus::SE2f icp(const point_cloud_2d &pc1, const point_cloud_2d &pc2, uint iter = 10) {
    auto count = pc1.size();
    Sophus::SE2f T21;
    for (int i = 0; i != iter; ++i) {
      Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
      Eigen::Vector3f g(0.0f, 0.0f, 0.0f);
      for (int i = 0; i != count; ++i) {
        auto &p1 = pc1.at(i);
        auto &p2 = pc2.at(i);
        auto p1_prime = T21 * p1;
        auto error = p1_prime - p2;
        Eigen::Matrix<float, 2, 3> J;
        J(0, 0) = 1, J(0, 1) = 0, J(0, 2) = -p1_prime(1);
        J(1, 0) = 0, J(1, 1) = 1, J(1, 2) = p1_prime(0);
        H += J.transpose() * J;
        g -= J.transpose() * error;
      }
      auto delta = H.ldlt().solve(g);
      T21 = Sophus::SE2f::exp(delta) * T21;
    }
    return T21;
  }

  static point_cloud_2d normalize(const point_cloud_2d &pc) {
    Eigen::Vector2f center(0.0f, 0.0f);
    for (const auto &elem : pc) {
      center += elem;
    }
    center /= pc.size();
    point_cloud_2d pc_new;
    for (const auto &elem : pc) {
      pc_new.push_back(elem - center);
    }
    return pc_new;
  }

  static Sophus::SE2f icp_no_binding(const point_cloud_2d &pc1, const point_cloud_2d &pc2, uint iter = 10) {
auto count = pc1.size();
auto pc1_new = normalize(pc1), pc2_new = normalize(pc2);
Sophus::SO2f R21;
for (int i = 0; i != iter; ++i) {
  float H = 0.0f;
  float g = 0.0f;
  for (int i = 0; i != count; ++i) {
    auto &p1 = pc1_new.at(i);
    auto p1_prime = R21 * p1;
    // find min dis point
    auto min_iter = std::min_element(pc2_new.cbegin(), pc2_new.cend(), [p1_prime](const Eigen::Vector2f &p2_i, const Eigen::Vector2f &p2_j) {
      auto p1_prime_t = ns_geo::Point2f(p1_prime(0), p1_prime(1));
      auto p2_i_t = ns_geo::Point2f(p2_i(0), p2_i(1));
      auto p2_j_t = ns_geo::Point2f(p2_j(0), p2_j(1));
      return ns_geo::distance(p1_prime_t, p2_i_t) < ns_geo::distance(p1_prime_t, p2_j_t);
    });
    auto &p2 = *min_iter;
    auto error = p1_prime - p2;
    Eigen::Vector2f J;
    J(0) = -p1_prime(1);
    J(1) = p1_prime(0);
    H += J.transpose() * J;
    g -= J.transpose() * error;
  }
  auto delta = g / H;
  R21 = Sophus::SO2f::exp(delta) * R21;
}
Eigen::MatrixXf A(2 * count, 2);
Eigen::VectorXf l(2 * count);
for (int i = 0; i != count; ++i) {
  auto trans = pc2.at(i) - R21 * pc1.at(i);
  A(2 * i + 0, 0) = 1, A(2 * i + 0, 1) = 0;
  A(2 * i + 1, 0) = 0, A(2 * i + 1, 1) = 1;
  l(2 * i + 0) = trans(0);
  l(2 * i + 1) = trans(1);
}
auto t21 = (A.transpose() * A).inverse() * A.transpose() * l;
Sophus::SE2f T21(R21.matrix(), Eigen::Vector2f(t21(0), t21(1)));
    return T21;
  }
} // namespace ns_st6

#endif