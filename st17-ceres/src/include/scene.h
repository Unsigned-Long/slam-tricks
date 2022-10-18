//
// Created by csl on 10/18/22.
//

#ifndef PNP_SCENE_H
#define PNP_SCENE_H

#include "colour.hpp"
#include "pose.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/visualization/pcl_visualizer.h"
#include "artwork/logger/logger.h"

namespace ns_st17 {
    struct CubePlane {
    protected:
        static ColourWheel _colourWheel;
    public:
        Posed _pose;
        float _xSpan;
        float _ySpan;
        float _zSpan;
        Colour _color;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr _features;

        CubePlane(float roll, float pitch, float yaw,
                  float dx, float dy, float dz,
                  float width, float height = 4.5f, float thickness = 0.1f,
                  int featureNum = 10, const Colour &color = CubePlane::_colourWheel.GetUniqueColour());

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    inline float DegreeToRadian(float deg) {
        static const float factor = M_PI / 180.0;
        return factor * deg;
    }

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
            // shot
            using std::placeholders::_1;
            _viewer->registerKeyboardCallback(
                    [this](auto &&PH1) { KeyBoardCallBack(std::forward<decltype(PH1)>(PH1)); }
            );

        }

        void RunSingleThread();

        void AddCamera(const std::string &name, const Posed &pose,
                       double r = 0.0, double g = 0.0, double b = 0.0,
                       float size = 2.0, float opacity = 1.0f, float lineWidth = 2.0f);

        void AddCubes(const std::vector<CubePlane> &planes, bool lineMode = false, float size = 1.0f);

        void AddLine(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2, const std::string &name);

        pcl::visualization::PCLVisualizer::Ptr &GetViewer();

    protected:

        void KeyBoardCallBack(const pcl::visualization::KeyboardEvent &ev);
    };
}


#endif //PNP_SCENE_H
