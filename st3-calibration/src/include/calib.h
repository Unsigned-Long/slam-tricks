#ifndef CALIB_H
#define CALIB_H

#include "cbcorners.h"
#include "eigen3/Eigen/Dense"
#include "helper.h"
#include "sophus/se3.hpp"

namespace ns_st3 {
  class CalibSolver {
    double cbSize;
    std::size_t rows, cols;

    std::size_t cbsCount;
    std::vector<CBPtsVec> cbsImgPts;
    std::vector<CBPtsVec> cbsObjPts;

    std::vector<Eigen::Matrix3d> HomoMats;

  public:
    double alpha, beta;
    double u0, v0;
    Eigen::Matrix3d intriMat;
    double k1, k2, k3, p1, p2;

    std::vector<Sophus::SE3d> imgPos;

  public:
    /**
     * @brief Construct a new Calib Solver object
     *
     * @param cornerDir the calib corners' dir
     * @param chessBoardSize the size of each block [Unit: m]
     */
    CalibSolver(const std::string &cornerDir, double chessBoardSize);

    void solve();

    void visualization();

    void testUndistortImg(const std::string &imgName);

  protected:
    void init(const std::string &cornerDir);

    void computeHomoMats();

    void reconstructIntriMat();

    void reconstructExtriMat();

    void computeDistCoeff();

    void totalOptimization();

  protected:
    Eigen::Matrix3d computeHomoMat(const CBPtsVec &imgPts, const CBPtsVec &objPts);

    cv::Point2d imgPt2NormPt(const cv::Point2d &imgPt);

    cv::Point2d normPt2ImgPt(const cv::Point2d &normPt);

    cv::Point2d distortNormPt(const cv::Point2d &npt_undist);
  };

  std::ostream &operator<<(std::ostream &os, const CalibSolver &solver);
} // namespace ns_st3

#endif