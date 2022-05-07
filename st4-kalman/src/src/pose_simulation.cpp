#include "pose_simulation.h"
#include "artwork/timer/timer.h"
#include "pcl-1.12/pcl/point_types.h"
#include "pcl-1.12/pcl/visualization/pcl_visualizer.h"
#include <functional>
#include <random>

namespace ns_st4 {
  float PoseItem::fps = 15.0f;

  std::ostream &operator<<(std::ostream &os, const PoseItem &obj) {
    os << "-- 'truth': " << obj.truth().log().transpose();
    os << "\n-- 'obs': " << obj.obs().log().transpose();
    return os;
  }

  std::vector<PoseItem> simulation(float fps, std::size_t num, float sigmaTrans, float sigmaAngle) {
    // fps
    PoseItem::fps = fps;
    // data vector
    std::vector<PoseItem> data;
    data.reserve(num);
    // create the first element
    data.emplace_back(Sophus::SE3f(), Sophus::SE3f());
    // random
    std::default_random_engine engine;
    std::normal_distribution<float> nTrans(0.0f, sigmaTrans);
    std::normal_distribution<float> nRot(0.0, sigmaAngle);
    for (int i = 1; i != num; ++i) {
      // truth
      auto updateRot = Eigen::AngleAxisf(M_PI / 180.0f, Eigen::Vector3f(0, 0, 1)) *
                       Eigen::AngleAxisf(M_PI / 180.0f, Eigen::Vector3f(0, 1, 0)) *
                       Eigen::AngleAxisf(M_PI / 180.0f, Eigen::Vector3f(1, 0, 0));
      auto updateTrans = Eigen::Vector3f(0.0f, 0.0f, 0.1f);

      Sophus::SE3f truth = Sophus::SE3f(updateRot, updateTrans) *
                           data.back().truth();
      // obs
      auto errorRot = Eigen::AngleAxisf(nRot(engine), Eigen::Vector3f(0, 0, 1)) *
                      Eigen::AngleAxisf(nRot(engine), Eigen::Vector3f(0, 1, 0)) *
                      Eigen::AngleAxisf(nRot(engine), Eigen::Vector3f(1, 0, 0));
      auto errorTrans = Eigen::Vector3f(nTrans(engine), nTrans(engine), nTrans(engine));
      Sophus::SE3f obs = Sophus::SE3f(errorRot, errorTrans) *
                         Sophus::SE3f(updateRot, updateTrans) *
                         data.back().obs();
      // create
      data.emplace_back(truth, obs);
    }
    return data;
  }

  void visualization(const std::vector<PoseItem> &data) {
    // point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr truthCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr obsCloud(new pcl::PointCloud<pcl::PointXYZ>());

    // view
    pcl::visualization::PCLVisualizer viewer("win");
    viewer.setSize(1000, 640);
    viewer.setBackgroundColor(0.9f, 0.9f, 0.9f);

    // the center
    Eigen::Isometry3f origin(Eigen::Quaternionf::Identity());
    origin.pretranslate(Eigen::Vector3f::Zero());
    viewer.addCoordinateSystem(1.0, Eigen::Affine3f(origin.affine()), "origin");

    // get coord and point
    auto getInfo = [](const Sophus::SE3f &pose) {
      Eigen::Vector3f pos(pose.translation());
      Eigen::Isometry3f coord(pose.unit_quaternion());
      coord.pretranslate(pos);
      pcl::PointXYZ p(pos(0), pos(1), pos(2));
      return std::make_pair(coord, p);
    };

    // timer
    ns_timer::Timer timer;
    float elapsed = 0.0f;
    const float targetElapsed = 1.0f / PoseItem::fps;

    for (int i = 0; i != data.size(); ++i) {
      timer.re_start();

      // current fame
      const auto &elem = data.at(i);

      auto truth = getInfo(elem.truth());
      auto obs = getInfo(elem.obs());

      // coord
      viewer.removeCoordinateSystem("truthCoord");
      viewer.removeCoordinateSystem("obsCoord");
      viewer.addCoordinateSystem(1.0, Eigen::Affine3f(truth.first.affine()), "truthCoord");
      viewer.addCoordinateSystem(1.0, Eigen::Affine3f(obs.first.affine()), "obsCoord");

      // point
      truthCloud->push_back(truth.second);
      obsCloud->push_back(obs.second);

      viewer.removePointCloud("truthCloud");
      viewer.removePointCloud("obsCloud");
      viewer.addPointCloud(truthCloud, "truthCloud");
      viewer.addPointCloud(obsCloud, "obsCloud");

      viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "truthCloud");
      viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "obsCloud");

      viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0, "truthCloud");
      viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.4, 0.4, 0.4, "obsCloud");

      viewer.setCameraPosition(truth.second.x + 5.0f, truth.second.y + 5.0f, truth.second.z + 5.0f,
                               0.0f, 0.0f, 0.0f);
      // display
      while (!viewer.wasStopped()) {
        viewer.spinOnce();
        if (i == data.size() - 1) {
          continue;
        }
        elapsed += timer.last_elapsed<ns_timer::DurationType::S>();
        if (elapsed > targetElapsed) {
          elapsed = 0.0f;
          break;
        }
      }
    }
  }
} // namespace na_st4
