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
    // params
    const std::size_t nc = 10, np = num;
    const float deltaTheta = 2 * M_PI * nc / (np - 1);
    const float Rc = 1.0f;
    const float Zc_w = 1.0f;
    const float deltaZ = 2.0f * Rc / (np - 1);
    const Eigen::Vector3f Po_w(0.0f, 0.0f, 0.0f);
    const Eigen::Vector3f Pc_w(0.0f, 0.0f, 1.0f);
    const Eigen::Vector3f Pt_w(0.0f, 0.0f, 2.0f);
    // data vector
    std::vector<PoseItem> data;
    data.reserve(np);
    data.emplace_back(Sophus::SE3f(), Sophus::SE3f());
    float lastTheta = 0.0f, lastZ = 0.0f;
    // random
    std::default_random_engine engine;
    std::normal_distribution<float> nTrans(0.0f, sigmaTrans);
    std::normal_distribution<float> nRot(0.0, sigmaAngle);
    for (int i = 1; i != np; ++i) {
      float theta_i = lastTheta + deltaTheta;
      float Zi_w = lastZ + deltaZ;
      float Ri = std::sqrt(Rc * Rc - std::pow(Zi_w - Zc_w, 2));
      float Xi_w = Ri * std::cos(theta_i);
      float Yi_w = Ri * std::sin(theta_i);
      Eigen::Vector3f twb(Xi_w, Yi_w, Zi_w);
      Eigen::Vector3f P_w = twb;

      // compute Rwb
      Eigen::Vector3f Pc_b(0.0f, 0.0f, 1.0f);

      Eigen::Vector3f PO = Po_w - P_w;
      Eigen::Vector3f PC = Pc_w - P_w;
      Eigen::Vector3f PT = Pt_w - P_w;

      float cos_theta1 = PO.dot(PC) / (PO.norm() * PC.norm());
      float sin_theta1 = std::sin(std::acos(cos_theta1));
      float cos_theta2 = PT.dot(PC) / (PT.norm() * PC.norm());
      float sin_theta2 = std::sin(std::acos(cos_theta2));

      Eigen::Vector3f Po_b(0.0f, PO.norm() * sin_theta1, PO.norm() * cos_theta1);
      Eigen::Vector3f Pt_b(0.0f, -PT.norm() * sin_theta2, PT.norm() * cos_theta2);

      Sophus::SO3f Rwb = computeRotation({Po_b, Pc_b, Pt_b}, {Po_w, Pc_w, Pt_w}, twb);

      // Rbw and tbw
      Sophus::SO3f Rbw = Rwb.inverse();
      Eigen::Vector3f tbw = -(Rbw * twb);

      Sophus::SE3f curTruth(Rbw, tbw);
      Sophus::SE3f lastTruth = data.back().truth();
      Sophus::SE3f truthUpdate_lc = curTruth * lastTruth.inverse();

      // error
      auto errorRot = Eigen::AngleAxisf(nRot(engine), Eigen::Vector3f(0, 0, 1)) *
                      Eigen::AngleAxisf(nRot(engine), Eigen::Vector3f(0, 1, 0)) *
                      Eigen::AngleAxisf(nRot(engine), Eigen::Vector3f(1, 0, 0));
      auto errorTrans = Eigen::Vector3f(nTrans(engine), nTrans(engine), nTrans(engine));

      Sophus::SE3f lastObs = data.back().obs();
      Sophus::SE3f curObs = Sophus::SE3f(errorRot, errorTrans) * truthUpdate_lc * lastObs;

      data.emplace_back(curTruth, curObs);

      // update
      lastTheta += deltaTheta;
      lastZ += deltaZ;
    }
    return data;
  }

  void visualization(const std::vector<PoseItem> &data) {
    // point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr truthCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr obsCloud(new pcl::PointCloud<pcl::PointXYZ>());
    truthCloud->reserve(data.size());
    obsCloud->reserve(data.size());

    // view
    pcl::visualization::PCLVisualizer::Ptr viewer = std::make_shared<pcl::visualization::PCLVisualizer>("win");
    viewer->setSize(1000, 640);
    viewer->setBackgroundColor(0.9f, 0.9f, 0.9f);

    // the center
    Eigen::Isometry3f origin(Eigen::Quaternionf::Identity());
    origin.pretranslate(Eigen::Vector3f::Zero());
    viewer->addCoordinateSystem(0.5, Eigen::Affine3f(origin.affine()), "origin");

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

      auto truth = getInfo(elem.truth().inverse());
      auto obs = getInfo(elem.obs().inverse());

      // coord

      // point
      truthCloud->push_back(truth.second);
      obsCloud->push_back(obs.second);

      viewer->removeCoordinateSystem("truthCoord");
      viewer->addCoordinateSystem(0.2, Eigen::Affine3f(truth.first.affine()), "truthCoord");
      viewer->removePointCloud("truthCloud");
      viewer->addPointCloud(truthCloud, "truthCloud");
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "truthCloud");
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0, "truthCloud");

      viewer->removeCoordinateSystem("obsCoord");
      viewer->addCoordinateSystem(0.2, Eigen::Affine3f(obs.first.affine()), "obsCoord");
      viewer->removePointCloud("obsCloud");
      viewer->addPointCloud(obsCloud, "obsCloud");
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "obsCloud");
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.4, 0.4, 0.4, "obsCloud");

      // set camera position
      viewer->setCameraPosition(truth.second.x + 3.0f, truth.second.y + 3.0f, truth.second.z + 3.0f,
                                0.0f, 0.0f, 0.0f);
      // display
      while (true) {
        if (i == data.size() - 1) {
          continue;
        }
        elapsed += timer.last_elapsed<ns_timer::DurationType::S>();
        if (elapsed > targetElapsed) {
          elapsed = 0.0f;
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      viewer->spinOnce();
      if (viewer->wasStopped()) {
        break;
      }
    }
  }

  Sophus::SO3f computeRotation(const std::vector<Eigen::Vector3f> &pts1,
                               const std::vector<Eigen::Vector3f> &pts2,
                               const Eigen::Vector3f &t21) {
    const std::size_t num = pts1.size();
    Sophus::SO3f rot21 = Sophus::SO3f();
    for (int i = 0; i != 20; ++i) {
      Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
      Eigen::Vector3f b = Eigen::Vector3f::Zero();
      for (int i = 0; i != num; ++i) {
        const Eigen::Vector3f &p1 = pts1.at(i);
        const Eigen::Vector3f &p2 = pts2.at(i);
        Sophus::SE3f trans21(rot21, t21);
        Eigen::Vector3f error = trans21 * p1 - p2;
        Eigen::MatrixX3f J = -Sophus::SO3f::hat(rot21 * p1);
        H += J.transpose() * J;
        b += -J.transpose() * error;
      }
      Eigen::Vector3f delta = H.ldlt().solve(b);
      rot21 = Sophus::SO3f::exp(delta) * rot21;
      if (delta.norm() < 1E-7) {
        break;
      }
    }
    return rot21;
  }

  float absTrajectoryError(const std::vector<Sophus::SE3f> &truth,
                           const std::vector<Sophus::SE3f> &esti) {
    float ate = 0.0;
    std::size_t num = truth.size();
    for (int i = 0; i != num; ++i) {
      const auto &truthPose = truth.at(i);
      const auto &estiPose = esti.at(i);
      ate += std::pow((truthPose.inverse() * estiPose).log().norm(), 2);
    }
    ate = std::sqrt(ate / num);
    return ate;
  }

} // namespace na_st4
