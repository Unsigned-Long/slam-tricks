//
// Created by csl on 10/16/22.
//

#include "scene.h"
#include "thread"

namespace ns_st16 {

    ColourWheel CubePlane::_colourWheel = ColourWheel(1.0f);

    CubePlane::CubePlane(float roll, float pitch, float yaw, float dx, float dy, float dz, float width, float height,
                         float thickness, int featureNum, const Colour &color)
            : _xSpan(thickness), _ySpan(width), _zSpan(height), _color(color),
              _features(new pcl::PointCloud<pcl::PointXYZRGB>) {
        // compute ths plane pose
        auto y = Eigen::AngleAxisd(DegreeToRadian(yaw), Eigen::Vector3d(0.0, 0.0, 1.0));
        auto p = Eigen::AngleAxisd(DegreeToRadian(pitch), Eigen::Vector3d(1.0, 0.0, 0.0));
        auto r = Eigen::AngleAxisd(DegreeToRadian(roll), Eigen::Vector3d(0.0, 1.0, 0.0));
        auto angleAxis = r * p * y;
        _pose = Posed::fromRt(angleAxis.matrix(), Eigen::Vector3d(dx, dy, dz));

        // generate random feature
        std::default_random_engine engine(std::chrono::system_clock::now().time_since_epoch().count());

        std::uniform_real_distribution<float> uy(-0.5f * _ySpan, 0.5f * _ySpan);
        std::uniform_real_distribution<float> uz(-0.5f * _zSpan, 0.5f * _zSpan);

        // generate feature
        if (featureNum > 0) {
            _features->resize(featureNum * 2);
            for (int i = 0; i < featureNum; ++i) {
                {
                    auto &pt = _features->at(i);
                    pt.r = pt.g = pt.b = 0.0f;

                    Eigen::Vector3f pos(0.5f * _xSpan, uy(engine), uz(engine));
                    Eigen::Vector3f result = _pose.se3().cast<float>() * pos;
                    pt.x = result(0), pt.y = result(1), pt.z = result(2);
                }
                {
                    auto &pt = _features->at(i * 2 + 1);
                    pt.r = pt.g = pt.b = 0.0f;

                    Eigen::Vector3f pos(-0.5f * _xSpan, uy(engine), uz(engine));
                    Eigen::Vector3f result = _pose.se3().cast<float>() * pos;
                    pt.x = result(0), pt.y = result(1), pt.z = result(2);
                }

            }
        }
    }

    void Scene::RunSingleThread() {
        while (!_viewer->wasStopped()) {
            // ms
            _viewer->spinOnce(100);
        }
    }

    void Scene::RunMultiThread() {

    }

    std::vector<ns_st16::Posed> Scene::ReadOdom(const std::string &filePath) {
        std::fstream file(filePath, std::ios::in);
        if (!file.is_open()) {
            throw std::runtime_error("[Reader::ReadOdom] open file failed.");
        }

        std::string strLine;

        std::vector<ns_st16::Posed> poseSeq;
        {
            // format
            std::getline(file, strLine);
            // element vertex num
            std::getline(file, strLine);
            auto num = std::stoul(Split(strLine, ' ')[2]);
            poseSeq.resize(num);
            // end header
            while (getline(file, strLine)) {
                if (strLine.size() >= 10 && strLine.substr(0, 10) == "end_header") {
                    break;
                }
            }
        }
        for (auto &pose: poseSeq) {
            getline(file, strLine);
            auto strVec = Split(strLine, ' ');

            auto timeStamp = std::stod(strVec[0]);

            Eigen::Quaterniond q;
            q.x() = std::stod(strVec[1]);
            q.y() = std::stod(strVec[2]);
            q.z() = std::stod(strVec[3]);
            q.w() = std::stod(strVec[4]);
            q.normalize();

            Eigen::Vector3d t;
            t(0) = std::stof(strVec[5]);
            t(1) = std::stof(strVec[6]);
            t(2) = std::stof(strVec[7]);

            pose = Posed::fromRt(q.toRotationMatrix(), t, timeStamp);
        }
        file.close();
        return poseSeq;
    }

    std::vector<std::string> Scene::Split(const std::string &str, char splitor, bool ignoreEmpty) {
        std::vector<std::string> vec;
        std::string elem;
        std::stringstream stream;
        stream << str;
        while (std::getline(stream, elem, splitor)) {
            if (elem.empty() && ignoreEmpty) {
                continue;
            }
            vec.push_back(elem);
        }
        return vec;
    }

    void Scene::AddPoseSeq(const std::string &name, const std::vector<ns_st16::Posed> &poseSeq,
                           int downSample, float size) {
        if (downSample < 1) {
            downSample = 1;
        }
        for (int i = 0; i < poseSeq.size(); i += downSample) {
            const auto &pose = poseSeq.at(i);
            Eigen::Isometry3d curToWorld(pose.so3.unit_quaternion());
            curToWorld.pretranslate(pose.t);

            _viewer->addCoordinateSystem(
                    size,
                    Eigen::Affine3f(curToWorld.cast<float>().affine()),
                    name + "-" + std::to_string(i)
            );
        }
    }

    void Scene::AddCubes(const std::vector<CubePlane> &planes, bool lineMode, float opacity) {
        for (int i = 0; i < planes.size(); ++i) {
            const auto &plane = planes.at(i);
            const auto &pose = plane._pose;
            const auto &color = plane._color;

            Eigen::Vector3f position = pose.t.cast<float>();
            Eigen::Quaternionf quat(pose.so3.matrix().cast<float>());

            _viewer->addCube(position, quat, plane._xSpan, plane._ySpan, plane._zSpan,
                             "CubePlane-" + std::to_string(i));

            if (lineMode) {
                _viewer->setShapeRenderingProperties(
                        pcl::visualization::RenderingProperties::PCL_VISUALIZER_REPRESENTATION,
                        pcl::visualization::RenderingRepresentationProperties::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                        "CubePlane-" + std::to_string(i)
                );
                _viewer->setShapeRenderingProperties(
                        pcl::visualization::RenderingProperties::PCL_VISUALIZER_LINE_WIDTH,
                        4.0, "CubePlane-" + std::to_string(i)
                );
            } else {
                _viewer->setShapeRenderingProperties(
                        pcl::visualization::RenderingProperties::PCL_VISUALIZER_OPACITY,
                        opacity, "CubePlane-" + std::to_string(i)
                );
            }

            _viewer->setShapeRenderingProperties(
                    pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR,
                    color.r, color.g, color.b, "CubePlane-" + std::to_string(i)
            );

            // feature
            _viewer->addPointCloud(plane._features, "Feature-" + std::to_string(i));
            _viewer->setPointCloudRenderingProperties(
                    pcl::visualization::RenderingProperties::PCL_VISUALIZER_POINT_SIZE, 6.0f,
                    "Feature-" + std::to_string(i)
            );
        }

    }

    void Scene::KeyBoardCallBack(const pcl::visualization::KeyboardEvent &ev) {
        if (ev.isAltPressed()) {
            std::int64_t curTimeStamp = std::chrono::system_clock::now().time_since_epoch().count();
            const std::string filename = "../scene/" + std::to_string(curTimeStamp) + ".png";
            _viewer->saveScreenshot(filename);
            LOG_INFO("the scene shot is saved to path: '", filename, "'.");
        }
    }

}
