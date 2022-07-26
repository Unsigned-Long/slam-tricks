#include "calib.h"

namespace ns_st3 {
  CalibSolver::CalibSolver(const std::string &cornerDir, double chessBoardSize) {
    init(cornerDir, chessBoardSize);
  }

  void CalibSolver::init(const std::string &cornerDir, double chessBoardSize) {
    auto files = filesInDir(cornerDir);
    for (const auto &file : files) {
      CBCorners cb = CBCorners::read(file);
      std::size_t rows = cb.rows();
      std::size_t cols = cb.cols();
      CBPtsVec objPtsVec(rows * cols);
      CBPtsVec imgPtsVec(rows * cols);
      for (int i = 0; i != rows; ++i) {
        for (int j = 0; j != cols; ++j) {
          std::size_t pos = i * cols + j;
          imgPtsVec[pos] = cb[i][j];
          objPtsVec[pos] = cv::Point2d(j, i) * chessBoardSize;
        }
      }
      cbsImgPts.push_back(imgPtsVec);
      cbsObjPts.push_back(objPtsVec);
    }
    cbsCount = files.size();
    HomoMats.resize(cbsCount);
  }

  void CalibSolver::solve() {
    computeHomoMats();

    reconstructIntriMat();

    reconstructExtriMat();
  }

  void CalibSolver::computeHomoMats() {
    for (int i = 0; i != cbsCount; ++i) {
      HomoMats[i] = computeHomoMat(cbsImgPts[i], cbsObjPts[i]);
    }
  }

  Eigen::Matrix3d CalibSolver::computeHomoMat(const CBPtsVec &imgPts, const CBPtsVec &objPts) {
    std::size_t size = imgPts.size();
    Eigen::MatrixXd A(size * 2, 9);
    A.setZero();
    for (int i = 0; i != size; ++i) {
      auto imgPt = imgPts[i];
      auto objPt = objPts[i];
      double x = objPt.x, y = objPt.y;
      double u = imgPt.x, v = imgPt.y;
      // first row
      A(i * 2 + 0, 0) = x;
      A(i * 2 + 0, 1) = y;
      A(i * 2 + 0, 2) = 1.0;
      A(i * 2 + 0, 6) = -u * x;
      A(i * 2 + 0, 7) = -u * y;
      A(i * 2 + 0, 8) = -u;
      // second row
      A(i * 2 + 1, 3) = x;
      A(i * 2 + 1, 4) = y;
      A(i * 2 + 1, 5) = 1.0;
      A(i * 2 + 1, 6) = -v * x;
      A(i * 2 + 1, 7) = -v * y;
      A(i * 2 + 1, 8) = -v;
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
    Eigen::Matrix<double, 9, 9> vMatrix = svd.matrixV();
    Eigen::Vector<double, 9> param = vMatrix.col(8);

    double a = param(0), b = param(1), c = param(2);
    double d = param(3), e = param(4), f = param(5);
    double g = param(6), h = param(7), i = param(8);

    Eigen::Matrix3d trans;
    trans(0, 0) = a, trans(0, 1) = b, trans(0, 2) = c;
    trans(1, 0) = d, trans(1, 1) = e, trans(1, 2) = f;
    trans(2, 0) = g, trans(2, 1) = h, trans(2, 2) = i;

    return trans;
  }

  void CalibSolver::reconstructIntriMat() {
    std::size_t size = HomoMats.size();
    Eigen::MatrixXd C(size * 2, 6);

    auto cofFunc = [](const Eigen::Matrix3d &hMat, std::size_t i, std::size_t j)
        -> Eigen::Matrix<double, 1, 6> {
      Eigen::Vector3d hi = hMat.col(i);
      Eigen::Vector3d hj = hMat.col(j);
      Eigen::Matrix<double, 1, 6> cofMat;
      cofMat(0, 0) = hi(0) * hj(0);
      cofMat(0, 1) = hi(1) * hj(0) + hi(0) * hj(1);
      cofMat(0, 2) = hi(2) * hj(0) + hi(0) * hj(2);
      cofMat(0, 3) = hi(1) * hj(1);
      cofMat(0, 4) = hi(2) * hj(1) + hi(1) * hj(2);
      cofMat(0, 5) = hi(2) * hj(2);
      return cofMat;
    };

    for (int i = 0; i != size; ++i) {
      Eigen::Matrix<double, 1, 6> cof12 = cofFunc(HomoMats[i], 0, 1);
      Eigen::Matrix<double, 1, 6> cof11 = cofFunc(HomoMats[i], 0, 0);
      Eigen::Matrix<double, 1, 6> cof22 = cofFunc(HomoMats[i], 1, 1);

      C.block(i * 2 + 0, 0, 1, 6) = cof12;
      C.block(i * 2 + 1, 0, 1, 6) = cof11 - cof22;
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(C, Eigen::ComputeFullV);
    Eigen::Matrix<double, 6, 6> vMatrix = svd.matrixV();
    Eigen::Vector<double, 6> param = vMatrix.col(5);

    double b11 = param(0), b12 = param(1), b13 = param(2);
    double b22 = param(3), b23 = param(4), b33 = param(5);

    v0 = (b12 * b13 - b11 * b23) / (b11 * b22 - b12 * b12);
    double lambda = b33 - (b13 * b13 + v0 * (b12 * b13 - b11 * b23)) / b11;
    alpha = std::sqrt(lambda / b11);
    beta = std::sqrt(lambda * b11 / (b11 * b22 - b12 * b12));
    gamma = -b12 * alpha * alpha * beta / lambda;
    u0 = gamma * v0 / beta - b13 * alpha * alpha / lambda;

    intriMat.setZero();
    intriMat(0, 0) = alpha;
    intriMat(0, 1) = gamma;
    intriMat(0, 2) = u0;
    intriMat(1, 1) = beta;
    intriMat(1, 2) = v0;
    intriMat(2, 2) = 1.0;
  }

  void CalibSolver::reconstructExtriMat() {
    std::size_t size = HomoMats.size();
    Eigen::Matrix3d intriMatInv = intriMat.inverse();

    imgPos.resize(size);

    for (int i = 0; i != size; ++i) {
      Eigen::Vector3d r1 = intriMatInv * HomoMats[i].col(0);
      Eigen::Vector3d r2 = intriMatInv * HomoMats[i].col(1);
      double lambda = 1.0 / (2.0 * r1.norm()) + 1.0 / (2.0 * r2.norm());
      // get r2
      r1.normalize(), r2.normalize();
      // get r3
      Eigen::Vector3d r3 = r1.cross(r2);
      // get r1
      r1 = r2.cross(r3);
      // get t
      Eigen::Vector3d t = lambda * intriMatInv * HomoMats[i].col(2);
      Eigen::Matrix3d rotMat;
      rotMat.col(0) = r1;
      rotMat.col(1) = r2;
      rotMat.col(2) = r3;
      int idx = 0;
      while (!Sophus::isOrthogonal(rotMat)) {
        if (idx % 2 == 0) {
          rotMat = adjustRotMat(rotMat.transpose()).transpose();
        } else {
          rotMat = adjustRotMat(rotMat);
        }
        ++idx;
      }
      imgPos[i] = Sophus::SE3d(rotMat, t);
    }
  }
} // namespace ns_st3
