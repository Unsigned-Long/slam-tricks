#ifndef CALIB_H
#define CALIB_H

#include "cbcorners.h"
#include "eigen3/Eigen/Dense"
#include "helper.h"

namespace ns_st3 {
  class CalibSolver {
    std::size_t cbsCount;
    std::vector<CBPtsVec> cbsImgPts;
    std::vector<CBPtsVec> cbsObjPts;

    std::vector<Eigen::Matrix3d> HomoMats;
    double alpha;
    double beta;
    double gamma;
    double u0;
    double v0;

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

  protected:
    Eigen::Matrix3d computeHomoMat(const CBPtsVec &imgPts, const CBPtsVec &objPts);
  };
} // namespace ns_st3

#endif