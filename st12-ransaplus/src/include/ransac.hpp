#ifndef RANSAC_HPP
#define RANSAC_HPP

#include "artwork/logger/logger.h"
#include "eigen3/Eigen/Dense"
#include <random>
#include <vector>

namespace ns_st12 {

  /**
   * @brief virtual class to solve ransac problem
   *
   * @tparam ElemType the element type
   * @tparam EigenParamVecSize the param size [for 'eigen' vector]
   * @tparam SubsetSize the minimum data set to fit the model
   */
  template <typename ElemType, int EigenParamVecSize, int SubsetSize>
  class Ransac {
  public:
    // the default constructor
    Ransac() = default;

  public:
    /**
     * @brief solve a ransac problem with mean shift
     *
     * @param data the dataset
     * @param modelParams the params for the final model ['eigen' vector type]
     * @param modelAvgResiual the average resiual for the final model
     * @param inlierResidualThd to decide an element is an outlier or an inlier
     * @param inliersRate the rate for inliners in the total dataset
     * @param iterCount the loop count for ransac
     * @return true if the result is good
     * @return false if some error happened when solving the ransac problem
     */
    bool solveWithMeanShift(const std::vector<ElemType> &data,
                            Eigen::Vector<double, EigenParamVecSize> &modelParams,
                            double &modelAvgResiual,
                            const double inlierResidualThd,
                            const double inliersRate = 0.3,
                            const std::size_t ransacIterCount = 20) {
      // initialize the model
      if (!solve(data, modelParams, modelAvgResiual, inlierResidualThd,
                 inliersRate, ransacIterCount)) {
        return false;
      }
      bool state = true;
      bool initialized = false;
      double lastAvgResiual;
      while (true) {
        // find the matching data in all data according to the current model
        std::vector<ElemType> inliers;
        for (const auto &elem : data) {
          double residual_t;
          if (!residual(modelParams, elem, residual_t)) {
            continue;
          }
          if (residual_t < inlierResidualThd) {
            inliers.push_back(elem);
          }
        }
        // fit the model again according to the coincidence point
        if (!fit(inliers, modelParams)) {
          state = false;
          break;
        }
        // compute the average resiual
        double avgResiual = 0.0;
        for (const auto &elem : inliers) {
          double residual_t;
          if (!residual(modelParams, elem, residual_t)) {
            continue;
          }
          avgResiual += residual_t;
        }
        avgResiual /= inliers.size();
        // update
        if (!initialized) {
          lastAvgResiual = avgResiual;
          initialized = true;
          continue;
        }
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

    /**
     * @brief solve a ransac problem
     *
     * @param data the dataset
     * @param modelParams the params for the final model ['eigen' vector type]
     * @param modelAvgResiual the average resiual for the final model
     * @param inlierResidualThd to decide an element is an outlier or an inlier
     * @param inliersRate the rate for inliners in the total dataset
     * @param iterCount the loop count for ransac
     * @return true if the result is good
     * @return false if some error happened when solving the ransac problem
     */
    bool solve(const std::vector<ElemType> &data,
               Eigen::Vector<double, EigenParamVecSize> &modelParams,
               double &modelAvgResiual,
               const double inlierResidualThd,
               const double inliersRate = 0.3,
               const std::size_t iterCount = 20) {
      // the random engine
      std::default_random_engine engine;
      // the variable for indicate whether the solver is initialized
      bool initialized = false;
      // the threshold [condition] for 'not bad' model
      std::size_t inliersThd = data.size() * inliersRate * 0.5;
      // start iteration
      for (int i = 0; i != iterCount; ++i) {
        // sampling the data of the smallest subset
        std::vector<ElemType> subset = samplingWoutReplace2(engine, data, SubsetSize);
        // fit model
        Eigen::Vector<double, EigenParamVecSize> params;
        if (!fit(subset, params)) {
          continue;
        }
        // find the matching data in all data according to the current model
        std::vector<ElemType> inliers;
        for (const auto &elem : data) {
          double residual_t;
          if (!residual(params, elem, residual_t)) {
            continue;
          }
          if (residual_t < inlierResidualThd) {
            inliers.push_back(elem);
          }
        }
        // check whether the current model is not too bad
        if (inliers.size() < inliersThd) {
          continue;
        }
        // fit the model again according to the coincidence point
        if (!fit(inliers, params)) {
          continue;
        }
        // compute the average resiual
        double avgResiual = 0.0;
        for (const auto &elem : inliers) {
          double residual_t;
          if (!residual(params, elem, residual_t)) {
            continue;
          }
          avgResiual += residual_t;
        }
        avgResiual /= inliers.size();
        // update the model
        if (!initialized || avgResiual < modelAvgResiual) {
          modelParams = params;
          modelAvgResiual = avgResiual;
          initialized = true;
        }
      }
      return initialized;
    }

  protected:
    virtual bool fit(const std::vector<ElemType> &subset,
                     Eigen::Vector<double, EigenParamVecSize> &params) const = 0;

    virtual bool residual(const Eigen::Vector<double, EigenParamVecSize> &params,
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