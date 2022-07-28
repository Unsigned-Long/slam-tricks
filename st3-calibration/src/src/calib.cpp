#include "calib.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "pcl-1.12/pcl/visualization/pcl_visualizer.h"
#include "struct_def.hpp"
#include "undistort_img.hpp"
#include <chrono>
#include <thread>

namespace ns_st3 {
  CalibSolver::CalibSolver(const std::string &cornerDir, double chessBoardSize)
      : cbSize(chessBoardSize) {
    init(cornerDir);
  }

  void CalibSolver::init(const std::string &cornerDir) {
    auto files = filesInDir(cornerDir);
    for (const auto &file : files) {
      CBCorners cb = CBCorners::read(file);
      rows = cb.rows();
      cols = cb.cols();
      CBPtsVec objPtsVec(rows * cols);
      CBPtsVec imgPtsVec(rows * cols);
      for (int i = 0; i != rows; ++i) {
        for (int j = 0; j != cols; ++j) {
          std::size_t pos = i * cols + j;
          imgPtsVec[pos] = cb[i][j];
          objPtsVec[pos] = cv::Point2d(j, i) * cbSize;
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

    totalOptimization();
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
    Eigen::MatrixXd C(size * 2, 5);

    auto cofFunc = [](const Eigen::Matrix3d &hMat, std::size_t i, std::size_t j)
        -> Eigen::Matrix<double, 1, 5> {
      Eigen::Vector3d hi = hMat.col(i);
      Eigen::Vector3d hj = hMat.col(j);
      Eigen::Matrix<double, 1, 5> cofMat;
      cofMat(0, 0) = hi(0) * hj(0);
      cofMat(0, 1) = hi(2) * hj(0) + hi(0) * hj(2);
      cofMat(0, 2) = hi(1) * hj(1);
      cofMat(0, 3) = hi(2) * hj(1) + hi(1) * hj(2);
      cofMat(0, 4) = hi(2) * hj(2);
      return cofMat;
    };

    for (int i = 0; i != size; ++i) {
      Eigen::Matrix<double, 1, 5> cof12 = cofFunc(HomoMats[i], 0, 1);
      Eigen::Matrix<double, 1, 5> cof11 = cofFunc(HomoMats[i], 0, 0);
      Eigen::Matrix<double, 1, 5> cof22 = cofFunc(HomoMats[i], 1, 1);

      C.block(i * 2 + 0, 0, 1, 5) = cof12;
      C.block(i * 2 + 1, 0, 1, 5) = cof11 - cof22;
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(C, Eigen::ComputeFullV);
    Eigen::Matrix<double, 5, 5> vMatrix = svd.matrixV();
    Eigen::Vector<double, 5> param = vMatrix.col(4);

    double b11 = param(0), b13 = param(1);
    double b22 = param(2), b23 = param(3), b33 = param(4);

    v0 = -b23 / b22;
    double lambda = b33 - (b13 * b13 - v0 * b11 * b23) / b11;
    alpha = std::sqrt(lambda / b11);
    beta = std::sqrt(lambda / b22);
    u0 = -b13 * alpha * alpha / lambda;

    intriMat.setZero();
    intriMat(0, 0) = alpha;
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

  void CalibSolver::visualization() {
    pcl::visualization::PCLVisualizer viewer("ViewSpace");

    // viewer.addCube()
    for (int i = 0; i != rows + 1; ++i) {
      for (int j = 0; j != cols + 1; ++j) {
        float gray;
        (i + j) % 2 == 0 ? gray = 1.0f : gray = 0.0f;
        viewer.addCube(j * cbSize - cbSize, (j + 1) * cbSize - cbSize,
                       i * cbSize - cbSize, (i + 1) * cbSize - cbSize,
                       0.0f, cbSize * 0.1f,
                       gray, gray, gray,
                       "Cube" + std::to_string(i) + std::to_string(j));
      }
    }

    // corners
    for (int i = 0; i != rows; ++i) {
      for (int j = 0; j != cols; ++j) {
        viewer.addSphere(pcl::PointXYZ(j * cbSize, i * cbSize, 0.0), cbSize * 0.1f,
                         1.0f, 1.0f, 0.0f, "Sphere" + std::to_string(i) + std::to_string(j));
      }
    }

    // coordinates
    viewer.addCoordinateSystem(cbSize, "Origin");

    Eigen::Vector3d center(0.0, 0.0, 0.0);
    for (int i = 0; i != cbsCount; ++i) {
      auto Twc = imgPos[i].inverse();
      auto quat = Twc.unit_quaternion();
      auto trans = Twc.translation();
      Eigen::Isometry3d coord(quat);
      coord.pretranslate(trans);
      // coordinates
      viewer.addCoordinateSystem(0.5f * cbSize, Eigen::Affine3f(coord.cast<float>().affine()),
                                 "Camera" + std::to_string(i));
      Eigen::Vector3d p1 = Twc * Eigen::Vector3d(0.0, 0.0, 0.0);
      Eigen::Vector3d p2 = Twc * Eigen::Vector3d(0.0, 0.0, cbSize * 0.5f);
      center += p1;
      // Cone
      pcl::ModelCoefficients cone_coeff;
      cone_coeff.values.resize(7); // We need 7 values
      cone_coeff.values[0] = p1(0);
      cone_coeff.values[1] = p1(1);
      cone_coeff.values[2] = p1(2);
      cone_coeff.values[3] = p2(0) - p1(0);
      cone_coeff.values[4] = p2(1) - p1(1);
      cone_coeff.values[5] = p2(2) - p1(2);
      cone_coeff.values[6] = 30.0; // degrees
      viewer.addCone(cone_coeff, "cone" + std::to_string(i));
    }

    // viewport
    center /= cbsCount;
    center *= 2.5;
    viewer.setCameraPosition(center(0), center(1), center(2),
                             cbSize * 0.5 * cols, cbSize * 0.5 * rows, 0.0,
                             0.0, -1.0, 0.0);

    while (!viewer.wasStopped()) {
      viewer.spinOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  cv::Point2d CalibSolver::imgPt2NormPt(const cv::Point2d &imgPt) {
    double ny = (imgPt.y - v0) / beta;
    double nx = (imgPt.x - u0) / alpha;
    return cv::Point2d(nx, ny);
  }

  cv::Point2d CalibSolver::normPt2ImgPt(const cv::Point2d &normPt) {
    double x = normPt.x, y = normPt.y;
    double u = alpha * x + u0;
    double v = beta * y + v0;
    return cv::Point2d(u, v);
  }

  cv::Point2d CalibSolver::distortNormPt(const cv::Point2d &npt_undist) {
    double x = npt_undist.x, y = npt_undist.y;
    double r2 = x * x + y * y, r4 = r2 * r2, r6 = r4 * r2;
    double x_dist = x * (1.0 + k1 * r2 + k2 * r4 + k3 * r6) +
                    2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
    double y_dist = y * (1.0 + k1 * r2 + k2 * r4 + k3 * r6) +
                    2.0 * p2 * x * y + p1 * (r2 + 2.0 * y * y);
    return cv::Point2d(x_dist, y_dist);
  }

  void CalibSolver::testUndistortImg(const std::string &imgName) {
    // structures for camera parameters
    ns_st0::CameraInnerParam innerParam(alpha, beta, u0, v0);
    ns_st0::CameraDistCoeff distCoff(k1, k2, k3, p1, p2);

    // read image
    cv::Mat distortedImg = cv::imread(imgName, cv::IMREAD_GRAYSCALE);

    cv::Mat undistortedImg = ns_st1::undistortImage(distortedImg, innerParam,
                                                    distCoff, Interpolation::NEAREST_NEIGHBOR);

    cv::namedWindow("distortedImg", cv::WINDOW_FREERATIO);
    cv::namedWindow("undistortedImg", cv::WINDOW_FREERATIO);
    cv::imshow("distortedImg", distortedImg);
    cv::imshow("undistortedImg", undistortedImg);
    cv::waitKey(0);
  }

  void CalibSolver::totalOptimization() {

    Eigen::VectorXd param(9 + 6 * cbsCount);
    // initialize

    // intri params
    param(0) = alpha, param(1) = beta, param(2) = u0, param(3) = v0;

    // dist params
    param(4) = param(5) = param(6) = param(7) = param(8) = 0.0;

    // pos params
    for (int i = 0; i != cbsCount; ++i) {
      param.block(9 + i * 6, 0, 6, 1) = imgPos[i].log();
    }

    for (int iter = 0; iter != 10; ++iter) {
      Eigen::VectorXd g(9 + 6 * cbsCount);
      Eigen::MatrixXd H(9 + 6 * cbsCount, 9 + 6 * cbsCount);
      g.setZero();
      H.setZero();

      // intri params
      alpha = param(0), beta = param(1), u0 = param(2), v0 = param(3);

      // dist params
      k1 = param(4), k2 = param(5), k3 = param(6);
      p1 = param(7), p2 = param(8);

      for (int i = 0; i != cbsCount; ++i) {
        const auto &curImgPts = cbsImgPts[i];
        const auto &curObjPts = cbsObjPts[i];
        const auto &curPos = Sophus::SE3d::exp(param.block(9 + i * 6, 0, 6, 1));

        for (int j = 0; j != curImgPts.size(); ++j) {
          // prepare
          const double X = curObjPts[j].x, Y = curObjPts[j].y, Z = 0.0;

          const Eigen::Vector3d unNormPt = curPos * Eigen::Vector3d(X, Y, Z);
          const double X_prime = unNormPt(0), Y_prime = unNormPt(1), Z_prime = unNormPt(2);

          const double xn = X_prime / Z_prime, yn = Y_prime / Z_prime;

          const cv::Point2d normPt_dist = distortNormPt(cv::Point2d(xn, yn));

          const double xd = normPt_dist.x, yd = normPt_dist.y;

          const cv::Point2d imgPt_pred = normPt2ImgPt(cv::Point2d(xd, yd));

          const cv::Point2d imgPt_real = curImgPts[j];

          // error
          Eigen::Vector2d error(imgPt_pred.x - imgPt_real.x, imgPt_pred.y - imgPt_real.y);

          // intri params
          Eigen::Matrix<double, 4, 2> J_intri = Eigen::Matrix<double, 4, 2>::Zero();
          J_intri(0, 0) = xd, J_intri(2, 0) = 1.0;
          J_intri(1, 1) = yd, J_intri(3, 1) = 1.0;

          // distort params
          const double r2 = xn * xn + yn * yn, r4 = r2 * r2, r6 = r4 * r2;
          Eigen::Matrix<double, 5, 2> J_dist = Eigen::Matrix<double, 5, 2>::Zero();
          J_dist(0, 0) = alpha * xn * r2, J_dist(0, 1) = beta * yn * r2;
          J_dist(1, 0) = alpha * xn * r4, J_dist(1, 1) = beta * yn * r4;
          J_dist(2, 0) = alpha * xn * r6, J_dist(2, 1) = beta * yn * r6;
          J_dist(3, 0) = 2.0 * alpha * xn * yn, J_dist(3, 1) = beta * (r2 + 2.0 * yn * yn);
          J_dist(4, 0) = alpha * (r2 + 2.0 * xn * xn), J_dist(4, 1) = 2.0 * beta * xn * yn;

          // pos param
          // [1]
          Eigen::Matrix2d ei_pd = Eigen::Matrix2d::Zero();
          ei_pd(0, 0) = alpha, ei_pd(1, 1) = beta;
          // [2]
          Eigen::Matrix2d pd_pn;
          pd_pn(0, 0) = (1.0 + k1 * r2 + k2 * r4 + k3 * r6) +
                        xn * (2.0 * k1 * xn + 4.0 * k2 * r2 * xn + 6.0 * k3 * r4 * xn) +
                        2.0 * p1 * yn + 6.0 * p2 * xn;
          pd_pn(0, 1) = 2.0 * p1 * xn;
          pd_pn(1, 0) = 2.0 * p2 * yn;
          pd_pn(1, 1) = (1.0 + k1 * r2 + k2 * r4 + k3 * r6) +
                        yn * (2.0 * k1 * yn + 4.0 * k2 * r2 * yn + 6.0 * k3 * r4 * yn) +
                        2.0 * p2 * xn + 6.0 * p1 * yn;
          // [3]
          Eigen::Matrix<double, 2, 3> pn_PPrime;
          const double Z_prime_inv = 1.0 / Z_prime, Z_prime_inv2 = Z_prime_inv * Z_prime_inv;
          pn_PPrime(0, 0) = Z_prime_inv;
          pn_PPrime(0, 1) = 0.0;
          pn_PPrime(0, 2) = -X_prime * Z_prime_inv2;
          pn_PPrime(1, 0) = 0.0;
          pn_PPrime(1, 1) = Z_prime_inv;
          pn_PPrime(1, 2) = -Y_prime * Z_prime_inv2;
          // [4]
          Eigen::Matrix<double, 3, 6> PPrime_pos;
          PPrime_pos.block(0, 0, 3, 3).setIdentity();
          PPrime_pos.block(0, 3, 3, 3) = -Sophus::SO3d::hat(unNormPt);
          // total [1, 2, 3, 4]
          Eigen::Matrix<double, 6, 2> J_pos = (ei_pd * pd_pn * pn_PPrime * PPrime_pos).transpose();

          // jacobian
          Eigen::MatrixXd J(9 + 6 * cbsCount, 2);
          J.setZero();
          J.block(0, 0, 4, 2) = J_intri;
          J.block(4, 0, 5, 2) = J_dist;
          J.block(9 + i * 6, 0, 6, 2) = J_pos;
          H += J * J.transpose();
          g -= J * error;
        }
      }

      Eigen::VectorXd update = H.ldlt().solve(g);
      param.block(0, 0, 9, 1) += update.block(0, 0, 9, 1);

      // pos update
      for (int i = 0; i != cbsCount; ++i) {
        param.block(9 + i * 6, 0, 6, 1) =
            (Sophus::SE3d::exp(update.block(9 + i * 6, 0, 6, 1)) *
             Sophus::SE3d::exp(param.block(9 + i * 6, 0, 6, 1)))
                .log();
      }

      if (update.norm() < 1E-8) {
        break;
      }
    }

    // assign

    // intri params
    alpha = param(0), beta = param(1), u0 = param(2), v0 = param(3);

    // dist params
    k1 = param(4), k2 = param(5), k3 = param(6);
    p1 = param(7), p2 = param(8);

    // pos params
    for (int i = 0; i != cbsCount; ++i) {
      imgPos[i] = Sophus::SE3d::exp(param.block(9 + i * 6, 0, 6, 1));
    }
  }

  std::ostream &operator<<(std::ostream &os, const CalibSolver &solver) {
    os << "{'alpha(fx)': " << solver.alpha << ", 'beta(fy)': " << solver.beta;
    os << ", 'u0(cx)': " << solver.u0 << ", 'v0(cy)': " << solver.v0;
    os << ", 'k1': " << solver.k1 << ", 'k2': " << solver.k2 << ", 'k3': " << solver.k3;
    os << ", 'p1': " << solver.p1 << ", 'p2': " << solver.p2 << "}";
    return os;
  }
} // namespace ns_st3
