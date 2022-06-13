#ifndef PARABOLA_HPP
#define PARABOLA_HPP

#include "artwork/geometry/point.hpp"
#define FORMAT_ARRAY
#include "artwork/logger/logger.h"
#include "eigen3/Eigen/Dense"

namespace ns_st7 {
  using parabola = std::array<float, 3>;
  static float computeVal(const parabola &para, float x) {
    auto [a, b, c] = para;
    return a * x * x + b * x + c;
  }
  static float computeGrad(const parabola &para, float x) {
    auto [a, b, c] = para;
    return 2.0f * a * x + b;
  }
  static parabola computePara(const ns_geo::Point2f &p1, const ns_geo::Point2f &p2, const ns_geo::Point2f &p3) {
    Eigen::Vector3f y(p1.y, p2.y, p3.y);
    Eigen::Matrix3f A;
    A(0, 0) = p1.x * p1.x, A(0, 1) = p1.x, A(0, 2) = 1.0f;
    A(1, 0) = p2.x * p2.x, A(1, 1) = p2.x, A(1, 2) = 1.0f;
    A(2, 0) = p3.x * p3.x, A(2, 1) = p3.x, A(2, 2) = 1.0f;
    auto ABC = A.inverse() * y;
    return {ABC(0), ABC(1), ABC(2)};
  }

  static std::pair<ns_geo::PointSet2f, ns_geo::PointSet2f> genData(const parabola &para) {
    auto [a, b, c] = para;
    float mid = -b / (2.0f * a);
    std::default_random_engine e;
    std::uniform_real_distribution<float> ux(mid - 2.0f, mid + 2.0f);
    std::normal_distribution<float> ey1(0.0f, 0.1f);
    std::normal_distribution<float> ey2(0.0f, 0.3f);
    ns_geo::PointSet2f allGood;
    ns_geo::PointSet2f withBad;
    for (int i = 0; i != 100; ++i) {
      float x = ux(e);
      ns_geo::Point2f p(x, computeVal(para, x + ey1(e)));
      allGood.push_back(p);
      withBad.push_back(p);
    }
    for (int i = 0; i != 100; ++i) {
      float x = ux(e);
      withBad.push_back(ns_geo::Point2f(x, ey2(e) + 4.0f));
    }
    return {allGood, withBad};
  }

  /**
   * @brief sampling the amples without replacement
   *
   * @param num the num of the samples to sampling
   * @param start the start index
   * @param end the end index
   * @param step the step
   * @attention range: [start, end](step) i.e. for [1, 5](2) -> pool: {1, 3, 5}
   * @return std::vector<std::size_t>
   */
  static std::default_random_engine engine;
  std::vector<std::size_t> samplingWoutReplace(std::size_t num, std::size_t start, std::size_t end, std::size_t step = 1) {
    // create the pool for sampling
    std::vector<std::size_t> idxPool((end - start) / step + 1);
    for (int i = 0; i != idxPool.size(); ++i) {
      idxPool.at(i) = start + i * step;
    }
    std::vector<std::size_t> res(num);
    // the engine
    for (std::size_t i = 0; i != num; ++i) {
      // generate the random index
      std::uniform_int_distribution<std::size_t> ui(0, idxPool.size() - 1);
      std::size_t ridx = ui(engine);
      // record it
      res.at(i) = idxPool.at(ridx);
      // remove it
      idxPool.at(ridx) = idxPool.back();
      idxPool.pop_back();
    }
    return res;
  }

  /**
   * @brief sampling the amples without replacement
   *
   * @tparam ElemType the element type
   * @param dataVec the data vector
   * @param num the num of the samples to sampling
   * @return std::vector<std::size_t>
   */
  template <typename ElemType>
  std::vector<std::size_t> samplingWoutReplace(const std::vector<ElemType> &dataVec, std::size_t num) {
    return samplingWoutReplace(num, 0, dataVec.size() - 1, 1);
  }

  struct Solver {
  public:
    static parabola leastSquare(const ns_geo::PointSet2f &pts) {
      const std::size_t PTS_NUM = pts.size();
      Eigen::MatrixXf B(PTS_NUM, 3), l(PTS_NUM, 1);
      for (int i = 0; i != PTS_NUM; ++i) {
        const auto &p = pts.at(i);
        B(i, 0) = p.x * p.x, B(i, 1) = p.x, B(i, 2) = 1;
        l(i, 0) = p.y;
      }
      auto X = (B.transpose() * B).inverse() * B.transpose() * l;
      return {X(0), X(1), X(2)};
    }

    static parabola gaussNewton(const ns_geo::PointSet2f &pts, const std::size_t iter) {
      const std::size_t PTS_NUM = pts.size();
      Eigen::Vector3f X(1.0f, 0.0f, 0.0f);
      for (int i = 0; i != iter; ++i) {
        Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
        Eigen::Vector3f g = Eigen::Vector3f::Zero();
        for (int j = 0; j != PTS_NUM; ++j) {
          const auto &p = pts.at(j);
          float error = computeVal({X(0), X(1), X(2)}, p.x) - p.y;
          Eigen::Vector3f J(p.x * p.x, p.x, 1.0f);
          H += J * J.transpose();
          g += -J * error;
        }
        auto delta = H.ldlt().solve(g);
        X += delta;
        if (delta.norm() < 1E-6) {
          break;
        }
      }
      return {X(0), X(1), X(2)};
    }

    static parabola ransac(const ns_geo::PointSet2f &pts, const std::size_t iter, float threshold) {
      const std::size_t PTS_NUM = pts.size();
      std::vector<std::pair<parabola, int>> results;
      for (int i = 0; i != iter; ++i) {
        auto idx = samplingWoutReplace(pts, 3);
        auto para = computePara(pts.at(idx[0]), pts.at(idx[1]), pts.at(idx[2]));
        int count = 0;
        for (int i = 0; i != PTS_NUM; ++i) {
          const auto &p = pts.at(i);
          float yDis = std::abs(computeVal(para, p.x) - p.y);
          if (yDis < threshold) {
            ++count;
          }
        }
        results.push_back({para, count});
      }
      auto maxIter = std::max_element(results.begin(), results.end(),
                                      [](const std::pair<parabola, int> &p1, const std::pair<parabola, int> &p2) {
                                        return p1.second < p2.second;
                                      });
      return maxIter->first;
    }
  };
} // namespace ns_st7

#endif