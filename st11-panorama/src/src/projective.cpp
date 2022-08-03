#include "projective.h"
#include "help.hpp"

namespace ns_st11 {
  Eigen::Matrix3d solveHomoMat(const std::vector<cv::Point2d> &pc1, const std::vector<cv::Point2d> &pc2) {
    CV_Assert(pc1.size() == pc2.size());
    CV_Assert(pc1.size() >= 4);

    std::size_t size = pc1.size();

    Eigen::MatrixXd A(2 * size, 9);
    A.setZero();

    for (int i = 0; i != size; ++i) {
      const auto &p1 = pc1[i], &p2 = pc2[i];
      double u1 = p1.x, v1 = p1.y;
      double u2 = p2.x, v2 = p2.y;
      //
      A(i * 2 + 0, 0) = u1;
      A(i * 2 + 0, 1) = v1;
      A(i * 2 + 0, 2) = 1.0;
      A(i * 2 + 0, 6) = -u2 * u1;
      A(i * 2 + 0, 7) = -u2 * v1;
      A(i * 2 + 0, 8) = -u2;
      //
      A(i * 2 + 1, 3) = u1;
      A(i * 2 + 1, 4) = v1;
      A(i * 2 + 1, 5) = 1.0;
      A(i * 2 + 1, 6) = -v2 * u1;
      A(i * 2 + 1, 7) = -v2 * v1;
      A(i * 2 + 1, 8) = -v2;
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
    Eigen::Matrix<double, 9, 9> vMatrix = svd.matrixV();

    Eigen::Vector<double, 9> param = vMatrix.col(8);
    double h11 = param(0), h12 = param(1), h13 = param(2);
    double h21 = param(3), h22 = param(4), h23 = param(5);
    double h31 = param(6), h32 = param(7), h33 = param(8);

    Eigen::Matrix3d H;

    H(0, 0) = h11, H(0, 1) = h12, H(0, 2) = h13;
    H(1, 0) = h21, H(1, 1) = h22, H(1, 2) = h23;
    H(2, 0) = h31, H(2, 1) = h32, H(2, 2) = h33;

    return H;
  }

  cv::Point2d projectHomoMat(const cv::Point2d &p, const Eigen::Matrix3d &hMat) {
    Eigen::Vector3d pVec(p.x, p.y, 1.0);
    Eigen::Vector3d vec = hMat * pVec;
    vec /= vec(2);
    return cv::Point2d(vec(0), vec(1));
  }

  Eigen::Matrix3d solveHomoMatByRANSAC(const std::vector<cv::Point2d> &pc1,
                                       const std::vector<cv::Point2d> &pc2,
                                       float errorThd,
                                       std::size_t iter) {
    CV_Assert(pc1.size() == pc2.size());
    CV_Assert(pc1.size() >= 4);
    CV_Assert(iter >= 1);
    CV_Assert(errorThd >= 0.0f);

    std::size_t size = pc1.size();

    std::default_random_engine engine;
    Eigen::Matrix3d H;
    int innerCount = 0;

    for (int i = 0; i != iter; ++i) {
      std::vector<size_t> idxVec = samplingWoutReplace(engine, pc1, 4);
      std::vector<cv::Point2d> pc1_t(4), pc2_t(4);
      for (int j = 0; j != idxVec.size(); ++j) {
        std::size_t idx = idxVec[j];
        pc1_t[j] = pc1[idx];
        pc2_t[j] = pc2[idx];
      }

      Eigen::Matrix3d hMat = solveHomoMat(pc1_t, pc2_t);
      int curInnerCount = 0;

      for (int k = 0; k != size; ++k) {
        const auto &p1 = pc1[k], &p2 = pc2[k];
        auto p2_pred = projectHomoMat(p1, hMat);
        float errorSquard = (p2.x - p2_pred.x) * (p2.x - p2_pred.x) +
                            (p2.y - p2_pred.y) * (p2.y - p2_pred.y);
        if (errorSquard < errorThd * errorThd) {
          ++curInnerCount;
        }
      }

      if (curInnerCount > innerCount) {
        innerCount = curInnerCount;
        H = hMat;
      }
    }
    LOG_VAR(innerCount, innerCount / float(size));
    return H;
  }
} // namespace ns_st11
