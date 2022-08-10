#include "artwork/geometry/point.hpp"
#include "artwork/logger/logger.h"
#include "ransac.hpp"

void genPts() {
  std::default_random_engine engine;
  std::normal_distribution<float> ni(3.0, 1.0);
  std::normal_distribution<float> no(10.0, 9.0);
  int outliersCount = 100, inliersCount = 50;
  ns_geo::PointSet2d ptsOutliers(outliersCount), ptsInliers(inliersCount);
  for (int i = 0; i != outliersCount; ++i) {
    ptsOutliers[i].x = no(engine);
    ptsOutliers[i].y = no(engine);
  }
  for (int i = 0; i != inliersCount; ++i) {
    ptsInliers[i].x = ni(engine);
    ptsInliers[i].y = ni(engine);
  }
  ptsOutliers.write("../data/outliers.txt", std::ios::out);
  ptsInliers.write("../data/inliers.txt", std::ios::out);
  ptsOutliers.write("../data/pts.txt", std::ios::out);
  ptsInliers.write("../data/pts.txt", std::ios::out | std::ios::app);
}

class RansacPosition : public ns_st12::Ransca<ns_geo::Point2d, 2, 1> {
public:
  using parent_type = ns_st12::Ransca<ns_geo::Point2d, 2, 1>;
  using parent_type::parent_type;

protected:
  virtual bool fit(const std::vector<ns_geo::Point2d> &subset,
                   Eigen::Vector<double, 2> &params) const override {
    params = Eigen::Vector2d(0.0, 0.0);
    for (const auto &pt : subset) {
      params += Eigen::Vector2d(pt.x, pt.y);
    }
    params /= subset.size();
    return true;
  }

  virtual bool residual(const Eigen::Vector<double, 2> &params,
                        const ns_geo::Point2d &inlier,
                        double &resiual) const override {
    resiual = Eigen::Vector2d(params(0) - inlier.x, params(1) - inlier.y).norm();
    return true;
  }
};

void ransacCase() {
  ns_geo::PointSet2d pts;
  pts.read("../data/pts.txt", std::ios::in);
  RansacPosition solver;
  Eigen::Vector2d modelParams;
  double modelAvgResiual;
  if (solver.solve(pts, modelParams, modelAvgResiual, 3, 50, 0.1)) {
    LOG_VAR(modelParams, modelAvgResiual);
    ns_geo::PointSet2d{{modelParams(0), modelParams(1)}}.write("../data/ransac.txt", std::ios::out);
  }
}

void lsqCase() {
  ns_geo::PointSet2d pts;
  pts.read("../data/pts.txt", std::ios::in);
  double x = 0.0, y = 0.0;
  for (const auto &pt : pts) {
    x += pt.x, y += pt.y;
  }
  x /= pts.size(), y /= pts.size();
  LOG_VAR(x, y);
  ns_geo::PointSet2d{{x, y}}.write("../data/lsq.txt", std::ios::out);
}

void ransacWithMeanShiftCase() {
  ns_geo::PointSet2d pts;
  pts.read("../data/pts.txt", std::ios::in);
  RansacPosition solver;
  Eigen::Vector2d modelParams;
  double modelAvgResiual;
  if (solver.solveWithMeanShift(pts, modelParams, modelAvgResiual, 3, 50, 0.1)) {
    LOG_VAR(modelParams, modelAvgResiual);
    ns_geo::PointSet2d{{modelParams(0), modelParams(1)}}.write("../data/ransac_ms.txt", std::ios::out);
  }
}

int main(int argc, char const *argv[]) {
  genPts();
  ransacCase();
  lsqCase();
  ransacWithMeanShiftCase();
  return 0;
}
