//
// Created by csl on 11/5/22.
//

#ifndef ST20_G2O_SIM_DATA_H
#define ST20_G2O_SIM_DATA_H

#include "slam-scene-viewer/scene_viewer.h"
#include "sophus/se3.hpp"

namespace ns_st20 {
    template<typename T>
    using aligned_vector = std::vector<T, Eigen::aligned_allocator<T>>;

    struct DataManager {
    public:
        struct OptPose {
        public:
            Sophus::SO3d SO3;
            Eigen::Vector3d POS;
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        struct LandMark {
        public:
            Eigen::Vector3d landmark;
            ns_viewer::Colour color;
            // indexed by camera index
            aligned_vector<std::pair<std::size_t, Eigen::Vector2d>> features;

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        aligned_vector<OptPose> cameraPoses;

        aligned_vector<LandMark> landmarks;

        // constraints
        OptPose firCameraPoseConstraint;
        OptPose lastCameraPoseConstraint;

    public:
        void Show(const std::string &dir) const {
            ns_viewer::SceneViewer viewer(dir);
            // cameras
            for (const auto &item: cameraPoses) {
                ns_viewer::Posed pose(item.SO3.matrix(), item.POS);
                viewer.AddCamera(pose.cast<float>(), ns_viewer::Colour(1.0f, 0.0f, 0.0f, 0.2f));
            }
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
            cloud->points.resize(landmarks.size());
            for (int i = 0; i < landmarks.size(); ++i) {
                const auto &landmark = landmarks.at(i);
                cloud->points.at(i).x = landmark.landmark(0);
                cloud->points.at(i).y = landmark.landmark(1);
                cloud->points.at(i).z = landmark.landmark(2);
                cloud->points.at(i).r = landmark.color.r * 255.0f;
                cloud->points.at(i).g = landmark.color.g * 255.0f;
                cloud->points.at(i).b = landmark.color.b * 255.0f;
                cloud->points.at(i).a = landmark.color.a * 255.0f;
            }
            viewer.AddFeatures(cloud);
            viewer.RunSingleThread();
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct Triangulation {
    public:
        Triangulation();
    };

    class ProblemScene : public ns_viewer::SceneViewer {
    public:
        using parent_type = ns_viewer::SceneViewer;

    private:
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr _landmarks;
        // landmark -> cameras
        std::vector<std::vector<std::pair<std::size_t, pcl::PointXY>>> _landmarkCameraVec;

        std::vector<ns_viewer::Posed> _cameras;
        // camera -> landmark
        std::vector<std::vector<std::size_t>> _cameraLandmarkVec;

        const double CAM_PLANE_HALF_WIDTH = 0.8;
        const double CAM_PLANE_HALF_HEIGHT = 0.6;

    public:
        explicit ProblemScene(std::string sceneShotSaveDir = "");

        void ShowCameraAt(std::size_t cameraIdx = 0);

        void ShowFeatureAt(std::size_t featureIdx = 0);

        void ShowCameras();

        void ShowFeatures();

        [[nodiscard]] DataManager Simulation() const;

        ~ProblemScene() override;

    protected:

        void CreateTrajectory();

        void CreateScene();

        void CreateMeasurements();
    };
}


#endif //ST20_G2O_SIM_DATA_H
