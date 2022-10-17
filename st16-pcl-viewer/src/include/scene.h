//
// Created by csl on 10/16/22.
//

#ifndef ST16_PCL_VIEWER_SCENE_H
#define ST16_PCL_VIEWER_SCENE_H


#include <utility>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "colour.hpp"
#include "pose.hpp"
#include "logger.h"

namespace ns_st16 {
    struct CubePlane {
    protected:
        static ColourWheel _colourWheel;
    public:
        Posed _pose;
        float _xSpan;
        float _ySpan;
        float _zSpan;
        Colour _color;

        CubePlane(float roll, float pitch, float yaw,
                  float dx, float dy, float dz,
                  float width, float height = 4.5f, float thickness = 0.1f,
                  const Colour &color = CubePlane::_colourWheel.GetUniqueColour())
                : _xSpan(thickness), _ySpan(width), _zSpan(height), _color(color) {
            auto y = Eigen::AngleAxisd(DegreeToRadian(yaw), Eigen::Vector3d(0.0, 0.0, 1.0));
            auto p = Eigen::AngleAxisd(DegreeToRadian(pitch), Eigen::Vector3d(1.0, 0.0, 0.0));
            auto r = Eigen::AngleAxisd(DegreeToRadian(roll), Eigen::Vector3d(0.0, 1.0, 0.0));
            auto angleAxis = r * p * y;
            _pose = Posed::fromRt(angleAxis.matrix(), Eigen::Vector3d(dx, dy, dz));
        }

    protected:
        static float DegreeToRadian(float deg) {
            static const float factor = M_PI / 180.0;
            return factor * deg;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    class Scene {
    private:
        pcl::visualization::PCLVisualizer::Ptr _viewer;
    public:

        explicit Scene(const Colour &background, bool addOriginCoord = true)
                : _viewer(new pcl::visualization::PCLVisualizer("Scene")) {
            _viewer->setBackgroundColor(background.r, background.g, background.b);
            // coordinates
            if (addOriginCoord) {
                _viewer->addCoordinateSystem(1.0, "Origin");
            }
        }

        void RunSingleThread();

        void RunMultiThread();

        void AddPoseSeq(const std::string &name, const std::vector<ns_st16::Posed> &poseSeq,
                        int downSample = 1, float size = 0.3);

        void AddCubes(const std::vector<CubePlane> &planes, bool lineMode = false, float opacity = 1.0f);

        static std::vector<ns_st16::Posed> ReadOdom(const std::string &filePath);

    protected:
        static std::vector<std::string>
        Split(const std::string &str, char splitor, bool ignoreEmpty = true);

    };
}


#endif //ST16_PCL_VIEWER_SCENE_H
