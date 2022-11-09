//
// Created by csl on 11/5/22.
//

#ifndef ST20_G2O_SIM_DATA_H
#define ST20_G2O_SIM_DATA_H

#include <utility>

#include "slam-scene-viewer/scene_viewer.h"
#include "sophus/se3.hpp"
#include "ceres/ceres.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

namespace ns_st20 {
    template<typename T>
    using aligned_vector = std::vector<T, Eigen::aligned_allocator<T>>;

    struct OptPose {
    public:
        Sophus::SO3d SO3;
        Eigen::Vector3d POS;

        OptPose(const Sophus::SO3d &so3 = Sophus::SO3d(), const Eigen::Vector3d &pos = Eigen::Vector3d::Zero())
                : SO3(so3), POS(pos) {}

        OptPose inverse() const {
            return OptPose(SO3.inverse(), -(SO3.inverse() * POS));
        }

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

    struct DataManager {
    public:

        aligned_vector<OptPose> cameraPoses;

        aligned_vector<LandMark> landmarks;

        // constraints
        OptPose frontCamPoseConstraint;
        OptPose backCamPoseConstraint;

        std::vector<ns_viewer::CubePlane> box;

        std::shared_ptr<ns_viewer::SceneViewer> viewer;

    public:
        void Run(const std::string &dir, const std::string &winName) {
            viewer = std::make_shared<ns_viewer::SceneViewer>(dir);
            viewer->SetWindowName(winName);
            viewer->RunMultiThread();
            DrawScene();
        }

        void DrawScene() {
            viewer->Lock();
            // cameras
            for (const auto &item: cameraPoses) {
                ns_viewer::Posed pose(item.SO3.matrix(), item.POS);
                viewer->AddCamera(pose.cast<float>(), ns_viewer::Colour(1.0f, 0.0f, 0.0f, 0.2f));
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
            viewer->AddFeatures(cloud);
            for (const auto &item: box) {
                viewer->AddCubePlane(item, true);
            }
            viewer->UnLock();
        }

        void RemoveScene() {
            viewer->Lock();
            viewer->RemoveEntities();
            viewer->UnLock();
        }

        static void Evaluate(const DataManager &data1, const DataManager &data2) {
            // TODO
        }

        cv::Mat Hessian() {
            constexpr int scale = 10;
            Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> jMat = Jacobian();
            Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> hMat = jMat.transpose() * jMat;
            cv::Mat img(hMat.rows(), hMat.cols(), CV_8UC3, cv::Scalar(255, 255, 255));
            LOG_PLAINTEXT("hessian matrix size: ", img.size)
            for (int i = 0; i < img.rows; ++i) {
                auto *row = img.ptr<uchar>(i);
                for (int j = 0; j < img.cols; ++j) {
                    if (hMat(i, j) == 1) {
                        row[j * img.channels() + 0] = 0;
                        row[j * img.channels() + 1] = 0;
                        row[j * img.channels() + 2] = 255;
                    }
                }
            }
            cv::resize(img, img, cv::Size(img.rows * scale, img.cols * scale), 0, 0,
                       cv::InterpolationFlags::INTER_NEAREST);
            for (int i = 0; i < img.rows; i += scale) {
                img.row(i).setTo(cv::Scalar(0));
            }
            img.row(img.rows - 1).setTo(cv::Scalar(0));
            for (int j = 0; j < img.cols; j += scale) {
                img.col(j).setTo(cv::Scalar(0));
            }
            img.col(img.cols - 1).setTo(cv::Scalar(0));
            return img;
        }

    protected:
        [[nodiscard]] Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> Jacobian() const {
            int residuals = 0;
            for (const auto &landmark: landmarks) {
                residuals += landmark.features.size();
            }
            /**
             *             | cameras | landmarks |
             * residual        ...        ...
             */
            Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> jMat(residuals, cameraPoses.size() + landmarks.size());
            jMat.setZero();
            residuals = 0;
            for (int i = 0; i < landmarks.size(); ++i) {
                const auto &landmark = landmarks.at(i);
                jMat(residuals, cameraPoses.size() + i) = 1;
                for (const auto &[cameraIdx, feature]: landmark.features) {
                    jMat(residuals, cameraIdx) = 1;
                    ++residuals;
                }
            }
            return jMat;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct Triangulation {
    private:
        const OptPose WtoC;
        const Eigen::Vector2d feature;

    public:
        Triangulation(OptPose WtoC, Eigen::Vector2d feature)
                : WtoC(std::move(WtoC)), feature(std::move(feature)) {}

        static auto Create(const OptPose &WtoC, const Eigen::Vector2d &feature) {
            return new ceres::AutoDiffCostFunction<Triangulation, 2, 3>(
                    new Triangulation(WtoC, feature));
        }

    public:
        template<typename T>
        bool operator()(const T *const pInW, T *residuals) const {
            // get params
            Eigen::Map<const Sophus::Vector3<T>> p(pInW);

            // trans
            Sophus::Vector3<T> pInC = WtoC.SO3 * p + WtoC.POS;
            pInC = pInC / pInC(2);

            // compute residuals
            Eigen::Map<Sophus::Vector2<T>> residualsMap(residuals);
            residualsMap = feature.template cast<T>() - pInC.block(0, 0, 2, 1);
            return true;
        }
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

        std::vector<ns_viewer::CubePlane> _box;

        const double CAM_PLANE_HALF_WIDTH = 0.8;
        const double CAM_PLANE_HALF_HEIGHT = 0.6;

        const int _featureCountPerFace;

    public:
        explicit ProblemScene(int featureCountPerFace = 100, std::string sceneShotSaveDir = "");

        void ShowCameraAt(std::size_t cameraIdx = 0);

        void ShowFeatureAt(std::size_t featureIdx = 0);

        void ShowCameras();

        void ShowFeatures();

        [[nodiscard]] DataManager
        Simulation(bool addNoise, double posNoise = 1.0, double angleNoise = 1.0) const;

        ~ProblemScene() override;

    protected:

        void CreateTrajectory();

        void CreateScene();

        void CreateMeasurements();
    };
}


#endif //ST20_G2O_SIM_DATA_H
