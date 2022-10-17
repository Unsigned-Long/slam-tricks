//
// Created by csl on 10/16/22.
//

#include "scene.h"
#include "thread"

namespace ns_st16 {

    ColourWheel CubePlane::_colourWheel = ColourWheel(1.0f);

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
        }
    }

}
