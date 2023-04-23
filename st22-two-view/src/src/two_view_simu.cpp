//
// Created by csl on 4/23/23.
//

#include "two_view_simu.h"
#include "artwork/logger/logger.h"

namespace ns_st22 {

    std::optional<Eigen::Vector2d>
    WorldLandmarkToImage(const Sophus::SE3d &SE3_FtoW, const Eigen::Matrix3d &kMat, const Eigen::Vector3d &lm,
                         int width,
                         int height) {
        Eigen::Vector3d lmInF = SE3_FtoW.inverse() * lm;
        if (lmInF(2) <= 0.0) {
            return {};
        }
        Eigen::Vector3d feature = kMat * Eigen::Vector3d(lmInF(0) / lmInF(2), lmInF(1) / lmInF(2), 1.0);
        if (feature(0) < 0.0 || feature(0) > width - 1 || feature(1) < 0.0 || feature(1) > height - 1) {
            return {};
        }
        return Eigen::Vector2d(feature(0), feature(1));
    }

    std::pair<std::vector<FeaturePair>, Sophus::SE3d>
    SimulateTwoViewObservations(const Eigen::Matrix3d &kMat, int width, int height, int lmSize, bool displayScene) {
        std::default_random_engine engine(std::chrono::steady_clock::now().time_since_epoch().count());
        std::uniform_real_distribution<double> u(0.0, 10.0);

        // generate landmarks
        std::vector<Eigen::Vector3d> lms(lmSize, Eigen::Vector3d::Zero());
        for (int i = 0; i < lmSize; ++i) {
            lms.at(i) = Eigen::Vector3d(u(engine), u(engine), u(engine));
        }

        // pose
        // frame 1
        Eigen::Matrix3d SO3_F1toW = Eigen::Matrix3d::Identity();
        Eigen::Vector3d POS_F1inW = Eigen::Vector3d(3.0, 5.0, 0.0);
        Sophus::SE3d SE3_F1toW(AdjustRotationMatrix(SO3_F1toW), POS_F1inW);

        // frame 2
        Eigen::Matrix3d SO3_F2toW = Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d(0.0, -1.0, 0.0)).toRotationMatrix();
        Eigen::Vector3d POS_F2inW = Eigen::Vector3d(7.0, 5.0, 0.0);
        Sophus::SE3d SE3_F2toW(AdjustRotationMatrix(SO3_F2toW), POS_F2inW);

        // simulate observations
        std::vector<FeaturePair> pairs;
        for (const auto &pInW: lms) {
            auto f1 = WorldLandmarkToImage(SE3_F1toW, kMat, pInW, width, height);
            auto f2 = WorldLandmarkToImage(SE3_F2toW, kMat, pInW, width, height);
            if (!f1 || !f2) {
                continue;
            } else {
                pairs.emplace_back(*f1, *f2, pInW, SE3_F1toW.inverse() * pInW, SE3_F2toW.inverse() * pInW);
            }
        }

        if (displayScene) {
            ns_viewer::SceneViewer viewer(
                    "/home/csl/CppWorks/artwork/slam-tricks/st22-two-view/img", "two view geometry"
            );

            viewer.AddCamera(ns_viewer::Posed(SO3_F1toW, POS_F1inW).cast<float>(), ns_viewer::Colour::Red());
            viewer.AddCamera(ns_viewer::Posed(SO3_F2toW, POS_F2inW).cast<float>(), ns_viewer::Colour::Green());

            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
            for (const auto &lm: lms) {
                pcl::PointXYZRGBA p;
                p.x = static_cast<float>(lm(0));
                p.y = static_cast<float>(lm(1));
                p.z = static_cast<float>(lm(2));
                auto c = ns_viewer::SceneViewer::GetUniqueColour();
                p.r = static_cast<std::uint8_t>(255.0 * c.r);
                p.g = static_cast<std::uint8_t>(255.0 * c.g);
                p.b = static_cast<std::uint8_t>(255.0 * c.b);
                p.a = 255;
                cloud->push_back(p);
            }
            viewer.AddFeatures(cloud);
            for (const auto &p: pairs) {
                viewer.AddLine(
                        p.pInW.cast<float>(), POS_F1inW.cast<float>(), ns_viewer::Colour::Red().WithAlpha(0.15f)
                );
                viewer.AddLine(
                        p.pInW.cast<float>(), POS_F2inW.cast<float>(), ns_viewer::Colour::Green().WithAlpha(0.15f)
                );
            }
            viewer.AddBox(
                    Eigen::Vector3f::Zero(), Eigen::Vector3f(10.0, 10.0, 10.0),
                    ns_viewer::Colour::Black().WithAlpha(0.15f)
            );

            viewer.RunSingleThread();
        }

        return {pairs, SE3_F1toW.inverse() * SE3_F2toW};
    }
}