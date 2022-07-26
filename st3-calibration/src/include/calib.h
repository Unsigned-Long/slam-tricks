#ifndef CALIB_H
#define CALIB_H

#include "cbcorners.h"
#include "eigen3/Eigen/Dense"
#include "helper.h"
#include "pcl-1.12/pcl/visualization/cloud_viewer.h"
#include "sophus/se3.hpp"

namespace ns_st3 {
  class CalibSolver {
    std::size_t cbsCount;
    std::vector<CBPtsVec> cbsImgPts;
    std::vector<CBPtsVec> cbsObjPts;

    std::vector<Eigen::Matrix3d> HomoMats;
    double alpha, beta, gamma;
    double u0, v0;
    Eigen::Matrix3d intriMat;

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

  protected:
    void init(const std::string &cornerDir, double chessBoardSize);

    void computeHomoMats();

    void reconstructIntriMat();

    void reconstructExtriMat();

    void visualization();

  protected:
    Eigen::Matrix3d computeHomoMat(const CBPtsVec &imgPts, const CBPtsVec &objPts);
  };
} // namespace ns_st3

#endif