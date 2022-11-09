//
// Created by csl on 11/5/22.
//

#include "sim_data.h"

#include <utility>
#include "algorithm"

namespace ns_st20 {

    ProblemScene::ProblemScene(int featureCountPerFace, std::string sceneShotSaveDir)
            : _featureCountPerFace(featureCountPerFace),
              ns_viewer::SceneViewer(std::move(sceneShotSaveDir)),
              _landmarks(new pcl::PointCloud<pcl::PointXYZRGBA>()) {
        SetWindowName("Problem Scene [Ground Truth]");
        CreateScene();
        CreateTrajectory();
        CreateMeasurements();
    }

    void ProblemScene::CreateScene() {
        std::vector<ns_viewer::CubePlane> planes{
                ns_viewer::CubePlane(0, 0, 180, 0, 5, 0, 10, 10, 0.001f),
                ns_viewer::CubePlane(0, 0, 0, 0, -5, 0, 10, 10, 0.001f),
                ns_viewer::CubePlane(0, 0, -90, 5, 0, 0, 10, 10, 0.001f),
                ns_viewer::CubePlane(0, 0, 90, -5, 0, 0, 10, 10, 0.001f),
                ns_viewer::CubePlane(0, 90, 0, 0, 0, 5, 10, 10, 0.001f),
                ns_viewer::CubePlane(0, -90, 0, 0, 0, -5, 10, 10, 0.001f),
        };
        for (auto &item: planes) {
            item.color = ns_viewer::Colour::Black();
            item.color.a = 0.1f;
            AddCubePlane(item, true);

            auto newFeatures = item.GenerateFeatures(_featureCountPerFace, ns_viewer::CubePlane::Face::FRONT, 0.4f);
            _landmarks->points.resize(_landmarks->size() + newFeatures->size());
            std::copy_n(
                    newFeatures->points.cbegin(), newFeatures->points.size(),
                    _landmarks->points.end() - static_cast<int>(newFeatures->size())
            );
        }
        _box = planes;
        AddFeatures(_landmarks);
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
                _cameras.push_back(pose);
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
        if (cameraIdx >= _cameras.size()) {
            return;
        }
        ns_viewer::SceneViewer::RunMultiThread();

        auto names = AddCamera(_cameras.at(cameraIdx).cast<float>(), ns_viewer::Colour::Red());
        const auto cameraPos = _cameras.at(cameraIdx).translation.cast<float>();

        for (const auto &featureIdx: _cameraLandmarkVec.at(cameraIdx)) {
            const auto feature = _landmarks->at(featureIdx);
            AddLine(
                    {feature.x, feature.y, feature.z}, {cameraPos(0), cameraPos(1), cameraPos(2)},
                    ns_viewer::Colour(feature.r / 255.0f, feature.g / 255.0f, feature.b / 255.0f, feature.a / 255.0f)
            );
        }

    }

    void ProblemScene::CreateMeasurements() {
        _landmarkCameraVec.resize(_landmarks->size());
        _cameraLandmarkVec.resize(_cameras.size());

        for (int cameraIdx = 0; cameraIdx < _cameras.size(); ++cameraIdx) {
            const auto &CtoW = _cameras.at(cameraIdx);
            auto WtoC = CtoW.inverse();
            for (int featureIdx = 0; featureIdx < _landmarks->size(); ++featureIdx) {
                const auto &pInW = _landmarks->at(featureIdx);
                const auto pInC = WtoC.trans(Eigen::Vector3d(pInW.x, pInW.y, pInW.z));
                if (pInC(2) < 0.0) {
                    continue;
                }
                const auto pInCamPlane = pInC / pInC(2);
                if (std::abs(pInCamPlane.x()) < CAM_PLANE_HALF_WIDTH &&
                    std::abs(pInCamPlane.y()) < CAM_PLANE_HALF_HEIGHT) {
                    pcl::PointXY p{};
                    p.x = pInCamPlane(0), p.y = pInCamPlane(1);

                    _landmarkCameraVec.at(featureIdx).push_back({cameraIdx, p});
                    _cameraLandmarkVec.at(cameraIdx).push_back(featureIdx);
                }
            }
        }

        auto featureCameraMinMax = std::minmax_element(
                _landmarkCameraVec.cbegin(), _landmarkCameraVec.cend(),
                [](const std::vector<std::pair<std::size_t, pcl::PointXY>> &idx1,
                   const std::vector<std::pair<std::size_t, pcl::PointXY>> &idx2) {
                    return idx1.size() < idx2.size();
                }
        );
        auto cameraFeatureMinMax = std::minmax_element(
                _cameraLandmarkVec.cbegin(), _cameraLandmarkVec.cend(),
                [](const std::vector<std::size_t> &idx1, const std::vector<std::size_t> &idx2) {
                    return idx1.size() < idx2.size();
                }
        );
        LOG_PLAINTEXT("Landmark count: ", _landmarks->points.size())
        LOG_PLAINTEXT(
                "Landmark tracked by cameras: min('idx': ", featureCameraMinMax.first - _landmarkCameraVec.cbegin(),
                ", 'num': ", featureCameraMinMax.first->size(), "), max('idx': ",
                featureCameraMinMax.second - _landmarkCameraVec.cbegin(),
                ", 'num': ", featureCameraMinMax.second->size(), ")."
        )
        LOG_ENDL()
        LOG_PLAINTEXT("Camera count: ", _cameras.size())
        LOG_PLAINTEXT(
                "Camera track landmarks: min('idx': ", cameraFeatureMinMax.first - _cameraLandmarkVec.cbegin(),
                ", 'num': ", cameraFeatureMinMax.first->size(), "), max('idx': ",
                cameraFeatureMinMax.second - _cameraLandmarkVec.cbegin(),
                ", 'num': ", cameraFeatureMinMax.second->size(), ")."
        )
    }

    void ProblemScene::ShowFeatureAt(std::size_t featureIdx) {
        if (featureIdx >= _landmarks->size()) {
            return;
        }
        ns_viewer::SceneViewer::RunMultiThread();

        const auto feature = _landmarks->at(featureIdx);
        for (const auto &cameraFeatureIdx: _landmarkCameraVec.at(featureIdx)) {
            const auto cameraPos = _cameras.at(cameraFeatureIdx.first).translation.cast<float>();
            AddLine(
                    {feature.x, feature.y, feature.z}, {cameraPos(0), cameraPos(1), cameraPos(2)},
                    ns_viewer::Colour(feature.r / 255.0f, feature.g / 255.0f, feature.b / 255.0f, feature.a / 255.0f)
            );
        }
    }

    void ProblemScene::ShowCameras() {
        ns_viewer::SceneViewer::RunMultiThread();
        std::vector<std::string> cameraNames, lineNames;
        for (int cameraIdx = 0; cameraIdx < _cameras.size() && !_viewer->wasStopped(); ++cameraIdx) {
            Lock();
            RemoveEntities({cameraNames, lineNames});
            cameraNames.clear(), lineNames.clear();

            // add camera and lines
            cameraNames = AddCamera(_cameras.at(cameraIdx).cast<float>(), ns_viewer::Colour::Red());
            const auto cameraPos = _cameras.at(cameraIdx).translation.cast<float>();
            for (const auto &featureIdx: _cameraLandmarkVec.at(cameraIdx)) {
                const auto feature = _landmarks->at(featureIdx);
                auto names = AddLine(
                        {feature.x, feature.y, feature.z}, {cameraPos(0), cameraPos(1), cameraPos(2)},
                        ns_viewer::Colour(feature.r / 255.0f, feature.g / 255.0f, feature.b / 255.0f,
                                          feature.a / 255.0f)
                );
                AppendNames(lineNames, names);
            }
            UnLock();
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }

    void ProblemScene::ShowFeatures() {
        ns_viewer::SceneViewer::RunMultiThread();

        std::uniform_int_distribution<std::size_t> idxGen(0, _landmarks->size() - 1);
        std::default_random_engine engine(std::chrono::system_clock::now().time_since_epoch().count());
        std::vector<std::string> lineNames;

        while (!_viewer->wasStopped()) {
            auto featureIdx = idxGen(engine);
            const auto feature = _landmarks->at(featureIdx);

            Lock();
            RemoveEntities(lineNames);
            lineNames.clear();

            for (const auto &cameraFeatureIdx: _landmarkCameraVec.at(featureIdx)) {
                const auto cameraPos = _cameras.at(cameraFeatureIdx.first).translation.cast<float>();
                auto names = AddLine(
                        {feature.x, feature.y, feature.z}, {cameraPos(0), cameraPos(1), cameraPos(2)},
                        ns_viewer::Colour(feature.r / 255.0f, feature.g / 255.0f, feature.b / 255.0f,
                                          feature.a / 255.0f)
                );
                AppendNames(lineNames, names);
            }
            UnLock();
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }

    DataManager ProblemScene::Simulation(bool addNoise, double posNoise, double angleNoise) const {
        DataManager manager;
        manager.frontCamPoseConstraint.SO3 = _cameras.front().rotation;
        manager.frontCamPoseConstraint.POS = _cameras.front().translation;

        manager.backCamPoseConstraint.SO3 = _cameras.back().rotation;
        manager.backCamPoseConstraint.POS = _cameras.back().translation;

        manager.box = _box;

        manager.landmarks.resize(this->_landmarks->size());
        for (int i = 0; i < _landmarks->size(); ++i) {
            manager.landmarks.at(i).landmark = Eigen::Vector3d(
                    _landmarks->at(i).x, _landmarks->at(i).y, _landmarks->at(i).z
            );
            manager.landmarks.at(i).color = ns_viewer::Colour(
                    _landmarks->at(i).r / 255.0f, _landmarks->at(i).g / 255.0f,
                    _landmarks->at(i).b / 255.0f, _landmarks->at(i).a / 255.0f
            );
            manager.landmarks.at(i).features.resize(_landmarkCameraVec.at(i).size());
            for (int j = 0; j < _landmarkCameraVec.at(i).size(); ++j) {
                manager.landmarks.at(i).features.at(j).first = _landmarkCameraVec.at(i).at(j).first;
                manager.landmarks.at(i).features.at(j).second = Eigen::Vector2d(
                        _landmarkCameraVec.at(i).at(j).second.x,
                        _landmarkCameraVec.at(i).at(j).second.y
                );
            }
        }

        std::default_random_engine engine(std::chrono::system_clock::now().time_since_epoch().count());
        std::normal_distribution<double> pNoise(0.0, posNoise);
        std::normal_distribution<double> aNoise(0.0, angleNoise / 180.0 * M_PI);

        manager.cameraPoses.resize(_cameras.size());
        for (int i = 0; i < _cameras.size(); ++i) {
            auto pose = _cameras.at(i);
            if (addNoise) {
                // noise
                Eigen::AngleAxisd a1(aNoise(engine), Eigen::Vector3d(0, 0, 1));
                Eigen::AngleAxisd a2(aNoise(engine), Eigen::Vector3d(0, 1, 0));
                Eigen::AngleAxisd a3(aNoise(engine), Eigen::Vector3d(1, 0, 0));

                manager.cameraPoses.at(i).SO3 = Eigen::Matrix3d(pose.rotation * (a1 * a2 * a3).toRotationMatrix());
                manager.cameraPoses.at(i).POS =
                        pose.translation + Eigen::Vector3d(pNoise(engine), pNoise(engine), pNoise(engine));
            } else {
                manager.cameraPoses.at(i).SO3 = pose.rotation;
                manager.cameraPoses.at(i).POS = pose.translation;
            }
        }

        manager.cameraPoses.front() = manager.frontCamPoseConstraint;
        manager.cameraPoses.back() = manager.backCamPoseConstraint;

        // triangulate the landmarks
        for (auto &landmark: manager.landmarks) {
            ceres::Problem problem;

            for (const auto &feature: landmark.features) {
                const auto WtoC = manager.cameraPoses.at(feature.first).inverse();
                auto costFunc = Triangulation::Create(WtoC, feature.second);
                problem.AddResidualBlock(costFunc, nullptr, landmark.landmark.data());
            }

            ceres::Solver::Options options;
            ceres::Solver::Summary summary;
            ceres::Solve(options, &problem, &summary);
        }

        return manager;
    }

}