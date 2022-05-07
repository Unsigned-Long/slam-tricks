#ifndef POSE_SIMULATION_H
#define POSE_SIMULATION_H

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "sophus/se3.hpp"
#include <iostream>
#include <memory>
#include <vector>

namespace ns_st4 {
  struct PoseItem {
  public:
    static float fps;

  private:
    /**
     * @brief the members
     */
    Sophus::SE3f _truth;
    Sophus::SE3f _obs;

  public:
    /**
     * @brief construct a new PoseItem object
     */
    PoseItem(const Sophus::SE3f &truth, const Sophus::SE3f &obs)
        : _truth(truth), _obs(obs) {}

    inline Sophus::SE3f &truth() { return this->_truth; }
    inline const Sophus::SE3f &truth() const { return this->_truth; }

    inline Sophus::SE3f &obs() { return this->_obs; }
    inline const Sophus::SE3f &obs() const { return this->_obs; }
  };
  /**
   * @brief override operator '<<' for type 'PoseItem'
   */
  std::ostream &operator<<(std::ostream &os, const PoseItem &obj);

  /**
   * @brief for data simulation
   *
   * @param fps frames per second
   * @param num the number of data
   * @param sigmaTrans the sigma for translation each time
   * @param sigmaAngle the sigma angle for roataion each time
   * @return std::vector<PoseItem> the data
   */
  std::vector<PoseItem> simulation(float fps = 15.0f,
                                   std::size_t num = 1000,
                                   float sigmaTrans = 0.01f,
                                   float sigmaAngle = M_PI / 1800.0f);

  void visualization(const std::vector<PoseItem> &data);

} // namespace name

#endif