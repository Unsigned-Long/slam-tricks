//
// Created by csl on 11/5/22.
//

#include "sim_data.h"

#include <utility>
#include "algorithm"

namespace ns_st20 {

    ProblemScene::ProblemScene(std::string sceneShotSaveDir)
            : ns_viewer::SceneViewer(std::move(sceneShotSaveDir)),
              _features(new pcl::PointCloud<pcl::PointXYZRGBA>()) {
        CreateScene();
        CreateTrajectory();
        CreateMeasurements();
    }

    void ProblemScene::CreateScene() {
        std::vector<ns_viewer::CubePlane> planes{
                ns_viewer::CubePlane(0, 0, 180, 0, 5, 0, 10, 10),
                ns_viewer::CubePlane(0, 0, 0, 0, -5, 0, 10, 10),
                ns_viewer::CubePlane(0, 0, -90, 5, 0, 0, 10, 10),
                ns_viewer::CubePlane(0, 0, 90, -5, 0, 0, 10, 10),
                ns_viewer::CubePlane(0, 90, 0, 0, 0, 5, 10, 10),
                ns_viewer::CubePlane(0, -90, 0, 0, 0, -5, 10, 10),
        };
        for (const auto &item: planes) {
            AddCubePlane(item, true);

            auto newFeatures = item.GenerateFeatures(100, ns_viewer::CubePlane::Face::FRONT, 0.6f);
            _features->points.resize(_features->size() + newFeatures->size());
            std::copy_n(
                    newFeatures->points.cbegin(), newFeatures->points.size(),
                    _features->points.end() - static_cast<int>(newFeatures->size())
            );
        }
        AddFeatures(_features);
    }

    void ProblemScene::CreateTrajectory() {
        const auto RADIUS = 3.0;
        const auto DELTA_Z = 0.02;
        const auto DELTA_DEG = 10.0;
        const auto DOWN_SAMPLE = 10;

        static const auto DEG_2_RAD = M_PI / 180.0;

        std::optional<pcl::PointXYZ> lastPos;

        int count = 0;
        for (double z = -RADIUS + 0.1, deg = 0.0; z < RADIUS - 0.1; ++count) {
            double rad = deg * DEG_2_RAD;
            double cosTheta = std::sqrt(RADIUS * RADIUS - z * z) / RADIUS;
            double x = RADIUS * cosTheta * std::cos(rad);
            double y = RADIUS * cosTheta * std::sin(rad);

            if (count % DOWN_SAMPLE == 0) {
                // add camera
                auto trans = Eigen::Vector3d(x, y, z);
                // x axis
                auto xAxis = Eigen::Vector3d(-trans(1), trans(0), 0.0).normalized();
                // z axis
                auto zAxis = (Eigen::Vector3d::Zero() - trans).normalized();
                // y axis
                auto yAxis = zAxis.cross(xAxis);
                // rot
                Eigen::Matrix3d rotMatrix;
                rotMatrix.col(0) = xAxis;
                rotMatrix.col(1) = yAxis;
                rotMatrix.col(2) = zAxis;

                ns_viewer::Posed pose(rotMatrix, trans);
                _cameraTraj.push_back(pose);
                AddCamera(pose.cast<float>(), ns_viewer::Colour(1.0f, 0.0f, 0.0f, 0.2f));
            }

            pcl::PointXYZ curPos(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
            if (lastPos) {
                AddLine(*lastPos, curPos, ns_viewer::Colour(0.0f, 1.0f, 0.0f, 0.2f), 2.0f);
                lastPos = curPos;
            } else {
                lastPos = curPos;
            }

            deg += DELTA_DEG;
            z += DELTA_Z;
        }

    }

    ProblemScene::~ProblemScene() = default;

    void ProblemScene::ShowCameraAt(std::size_t cameraIdx) {
        if (cameraIdx >= _cameraTraj.size()) {
            return;
        }
        ns_viewer::SceneViewer::RunMultiThread();

        auto names = AddCamera(_cameraTraj.at(cameraIdx).cast<float>(), ns_viewer::Colour::Red());
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        const auto cameraPos = _cameraTraj.at(cameraIdx).translation.cast<float>();

        for (const auto &featureIdx: _cameraFeatureVec.at(cameraIdx)) {
            const auto feature = _features->at(featureIdx);
            AddLine(
                    {feature.x, feature.y, feature.z}, {cameraPos(0), cameraPos(1), cameraPos(2)},
                    ns_viewer::Colour(feature.r / 255.0f, feature.g / 255.0f, feature.b / 255.0f, feature.a / 255.0f)
            );
        }

    }

    void ProblemScene::CreateMeasurements() {
        _featureCamerasVec.resize(_features->size());
        _cameraFeatureVec.resize(_cameraTraj.size());

        for (int cameraIdx = 0; cameraIdx < _cameraTraj.size(); ++cameraIdx) {
            const auto &CtoW = _cameraTraj.at(cameraIdx);
            auto WtoC = CtoW.inverse();
            for (int featureIdx = 0; featureIdx < _features->size(); ++featureIdx) {
                const auto &pInW = _features->at(featureIdx);
                const auto pInC = WtoC.trans(Eigen::Vector3d(pInW.x, pInW.y, pInW.z));
                if (pInC(2) < 0.0) {
                    continue;
                }
                const auto pInCamPlane = pInC / pInC(2);
                if (std::abs(pInCamPlane.x()) < CAM_PLANE_HALF_WIDTH &&
                    std::abs(pInCamPlane.y()) < CAM_PLANE_HALF_HEIGHT) {
                    _featureCamerasVec.at(featureIdx).push_back(cameraIdx);
                    _cameraFeatureVec.at(cameraIdx).push_back(featureIdx);
                }
            }
        }

        auto featureCameraMinMax = std::minmax_element(
                _featureCamerasVec.cbegin(), _featureCamerasVec.cend(),
                [this](const std::vector<std::size_t> &idx1, const std::vector<std::size_t> &idx2) {
                    return idx1.size() < idx2.size();
                }
        );
        auto cameraFeatureMinMax = std::minmax_element(
                _cameraFeatureVec.cbegin(), _cameraFeatureVec.cend(),
                [this](const std::vector<std::size_t> &idx1, const std::vector<std::size_t> &idx2) {
                    return idx1.size() < idx2.size();
                }
        );
        LOG_PLAINTEXT("Feature count: ", _features->points.size())
        LOG_PLAINTEXT(
                "Feature tracked by cameras: min('idx': ", featureCameraMinMax.first - _featureCamerasVec.cbegin(),
                ", 'num': ", featureCameraMinMax.first->size(), "), max('idx': ",
                featureCameraMinMax.second - _featureCamerasVec.cbegin(),
                ", 'num': ", featureCameraMinMax.second->size(), ")."
        )

        LOG_PLAINTEXT("Camera count: ", _cameraTraj.size())
        LOG_PLAINTEXT(
                "Camera track features: min('idx': ", cameraFeatureMinMax.first - _cameraFeatureVec.cbegin(),
                ", 'num': ", cameraFeatureMinMax.first->size(), "), max('idx': ",
                cameraFeatureMinMax.second - _cameraFeatureVec.cbegin(),
                ", 'num': ", cameraFeatureMinMax.second->size(), ")."
        )
    }

    void ProblemScene::ShowFeatureAt(std::size_t featureIdx) {
        if (featureIdx >= _features->size()) {
            return;
        }
        ns_viewer::SceneViewer::RunMultiThread();

        const auto feature = _features->at(featureIdx);
        for (const auto &cameraIdx: _featureCamerasVec.at(featureIdx)) {
            const auto cameraPos = _cameraTraj.at(cameraIdx).translation.cast<float>();
            AddLine(
                    {feature.x, feature.y, feature.z}, {cameraPos(0), cameraPos(1), cameraPos(2)},
                    ns_viewer::Colour(feature.r / 255.0f, feature.g / 255.0f, feature.b / 255.0f, feature.a / 255.0f)
            );
        }
    }

    void ProblemScene::Show() {
        ns_viewer::SceneViewer::RunMultiThread();
    }

}