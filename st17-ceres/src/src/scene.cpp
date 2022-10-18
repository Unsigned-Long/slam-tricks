//
// Created by csl on 10/18/22.
//

#include "scene.h"

namespace ns_st17 {

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
            _features->resize(featureNum);
            for (int i = 0; i < featureNum; ++i) {
                {
                    auto &pt = _features->at(i);
                    auto c = _colourWheel.GetUniqueColour();
                    pt.r = c.r * 255.0f, pt.g = c.g * 255.0f, pt.b = c.b * 255.f;

                    Eigen::Vector3f pos(0.0f * _xSpan, uy(engine), uz(engine));
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

    void Scene::AddCamera(const std::string &name, const Posed &pose,
                          double r, double g, double b, float size, float opacity, float lineWidth) {
        Eigen::Isometry3d CtoW(pose.so3.unit_quaternion());
        CtoW.pretranslate(pose.t);
        _viewer->addCoordinateSystem(
                0.4,
                Eigen::Affine3f(CtoW.cast<float>().affine()),
                name
        );
        float left = -0.2f * size, right = 0.2f * size, top = -0.15f * size, bottom = 0.15f * size, front = 0.2f * size;

        pcl::PointXYZ leftBottom = {left, bottom, front};
        pcl::PointXYZ leftTop = {left, top, front};
        pcl::PointXYZ rightTop = {right, top, front};
        pcl::PointXYZ rightBottom = {right, bottom, front};
        pcl::PointXYZ center = {0.0, 0.0, 0.0};

        pcl::PointCloud<pcl::PointXYZ>::Ptr polygon1(new pcl::PointCloud<pcl::PointXYZ>);
        polygon1->push_back(leftBottom);
        polygon1->push_back(leftTop);
        polygon1->push_back(rightTop);
        polygon1->push_back(rightBottom);

        pcl::PointCloud<pcl::PointXYZ>::Ptr polygon2(new pcl::PointCloud<pcl::PointXYZ>);
        polygon2->push_back(center);
        polygon2->push_back(leftBottom);
        polygon2->push_back(leftTop);

        pcl::PointCloud<pcl::PointXYZ>::Ptr polygon3(new pcl::PointCloud<pcl::PointXYZ>);
        polygon3->push_back(center);
        polygon3->push_back(rightBottom);
        polygon3->push_back(rightTop);

        std::vector<std::pair<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>> data{
                {"front", polygon1},
                {"left",  polygon2},
                {"right", polygon3}
        };

        for (const auto &item: data) {
            for (auto &p: (*item.second)) {
                auto result = CtoW * Eigen::Vector3d(p.x, p.y, p.z);
                p.x = result(0), p.y = result(1), p.z = result(2);
            }
            _viewer->addPolygon<pcl::PointXYZ>(item.second, r, g, b, name + item.first);
            _viewer->setShapeRenderingProperties(
                    pcl::visualization::RenderingProperties::PCL_VISUALIZER_LINE_WIDTH, lineWidth, name + item.first
            );
            _viewer->setShapeRenderingProperties(
                    pcl::visualization::RenderingProperties::PCL_VISUALIZER_OPACITY,
                    opacity, name + item.first
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
                    pcl::visualization::RenderingProperties::PCL_VISUALIZER_POINT_SIZE, 10.0f,
                    "Feature-" + std::to_string(i)
            );
        }

    }

    void Scene::KeyBoardCallBack(const pcl::visualization::KeyboardEvent &ev) {
        if (ev.isAltPressed()) {
            std::int64_t curTimeStamp = std::chrono::system_clock::now().time_since_epoch().count();
            const std::string filename = "../scene/" + std::to_string(curTimeStamp) + ".png";
            _viewer->saveScreenshot(filename);
            LOG_INFO("the scene shot is saved to path: '", filename, "'.")
        }
    }

    void Scene::AddLine(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2, const std::string &name) {
        auto c = Colour::Black();
        _viewer->addLine(p1, p2, c.r, c.g, c.b, name);
        _viewer->setShapeRenderingProperties(
                pcl::visualization::RenderingProperties::PCL_VISUALIZER_OPACITY,
                0.3, name
        );
    }

    pcl::visualization::PCLVisualizer::Ptr &Scene::GetViewer() {
        return _viewer;
    }
}