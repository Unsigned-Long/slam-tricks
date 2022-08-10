#ifndef RANSAC_HPP
#define RANSAC_HPP

#include "artwork/logger/logger.h"
#include "eigen3/Eigen/Dense"
#include <random>
#include <vector>

namespace ns_st12 {

  template <typename ElemType, int EigenVecSize, int SubsetSize>
  class Ransca {
  public:
    Ransca() = default;

  public:
    bool solveWithMeanShift(const std::vector<ElemType> &data,
                            Eigen::Vector<double, EigenVecSize> &modelParams,
                            double &modelAvgResiual,
                            const double inlierResidualThd,
                            const std::size_t ransacIterCount = 20,
                            const double validModelInliersCountThd = 0.3) {
      if (!solve(data, modelParams, modelAvgResiual, inlierResidualThd,
                 ransacIterCount, validModelInliersCountThd)) {
        return false;
      }
      bool state = true;
      bool initialized = false;
      double lastAvgResiual;
      while (true) {
        std::vector<ElemType> inliers;
        for (const auto &elem : data) {
          double residual_t;
          if (!residual(modelParams, elem, residual_t)) {
            state = false;
            break;
          }
          if (residual_t < inlierResidualThd) {
            inliers.push_back(elem);
          }
        }
        if (!state) {
          break;
        }
        if (!fit(inliers, modelParams)) {
          state = false;
          break;
        }
        double avgResiual = 0.0;
        for (const auto &elem : inliers) {
          double residual_t;
          if (!residual(modelParams, elem, residual_t)) {
            state = false;
            break;
          }
          avgResiual += residual_t;
        }
        if (!state) {
          break;
        }
        avgResiual /= inliers.size();
        if (!initialized) {
          lastAvgResiual = avgResiual;
          initialized = true;
          continue;
        }
        LOG_VAR(avgResiual);
        double delta = std::abs(lastAvgResiual - avgResiual);
        if (delta < 1E-8) {
          lastAvgResiual;
          break;
        } else {
          lastAvgResiual = avgResiual;
        }
      }
      modelAvgResiual = lastAvgResiual;
      return state && initialized;
    }

    bool solve(const std::vector<ElemType> &data,
               Eigen::Vector<double, EigenVecSize> &modelParams,
               double &modelAvgResiual,
               const double inlierResidualThd,
               const std::size_t iterCount = 20,
               const double validModelInliersCountThd = 0.3) {
      bool state = true;
      std::default_random_engine engine;
      bool initialized = false;
      std::size_t inliersThd = data.size() * validModelInliersCountThd;
      for (int i = 0; i != iterCount; ++i) {
        std::vector<ElemType> subset = samplingWoutReplace2(engine, data, SubsetSize);
        Eigen::Vector<double, EigenVecSize> params;
        if (!fit(subset, params)) {
          state = false;
          break;
        }
        std::vector<ElemType> inliers;
        for (const auto &elem : data) {
          double residual_t;
          if (!residual(params, elem, residual_t)) {
            state = false;
            break;
          }
          if (residual_t < inlierResidualThd) {
            inliers.push_back(elem);
          }
        }
        if (!state) {
          break;
        }
        if (inliers.size() < inliersThd) {
          continue;
        }
        if (!fit(inliers, params)) {
          state = false;
          break;
        }
        double avgResiual = 0.0;
        for (const auto &elem : inliers) {
          double residual_t;
          if (!residual(params, elem, residual_t)) {
            state = false;
            break;
          }
          avgResiual += residual_t;
        }
        if (!state) {
          break;
        }
        avgResiual /= inliers.size();
        if (!initialized || avgResiual < modelAvgResiual) {
          modelParams = params;
          modelAvgResiual = avgResiual;
          initialized = true;
        }
      }
      return state && initialized;
    }

  protected:
    virtual bool fit(const std::vector<ElemType> &subset,
                     Eigen::Vector<double, EigenVecSize> &params) const = 0;

    virtual bool residual(const Eigen::Vector<double, EigenVecSize> &params,
                          const ElemType &inlier,
                          double &resiual) const = 0;

  private:
    /**
     * @brief sampling the amples without replacement
     *
     * @param num the num of the samples to sampling
     * @param engine the random engine
     * @param start the start index
     * @param end the end index
     * @param step the step
     * @attention range: [start, end](step) i.e. for [1, 5](2) -> pool: {1, 3, 5}
     * @return std::vector<std::size_t>
     */
    std::vector<std::size_t> samplingWoutReplace(std::default_random_engine &engine,
                                                 std::size_t num,
                                                 std::size_t start,
                                                 std::size_t end,
                                                 std::size_t step = 1) {
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
     * @param engine the random engine
     * @param dataVec the data vector
     * @param num the num of the samples to sampling
     * @return std::vector<std::size_t>
     */
    std::vector<std::size_t> samplingWoutReplace(std::default_random_engine &engine,
                                                 const std::vector<ElemType> &dataVec,
                                                 std::size_t num) {
      return samplingWoutReplace(engine, num, 0, dataVec.size() - 1, 1);
    }

    /**
     * @brief sampling the amples without replacement
     *
     * @tparam ElemType the element type
     * @param engine the random engine
     * @param dataVec the data vector
     * @param num the num of the samples to sampling
     * @return std::vector<ElemType>
     */
    std::vector<ElemType> samplingWoutReplace2(std::default_random_engine &engine,
                                               const std::vector<ElemType> &dataVec,
                                               std::size_t num) {
      std::vector<std::size_t> res = samplingWoutReplace(engine, dataVec, num);
      std::vector<ElemType> samples(num);
      for (int i = 0; i != num; ++i) {
        samples.at(i) = dataVec.at(res.at(i));
      }
      return samples;
    }
  };
} // namespace ns_st12

#endif