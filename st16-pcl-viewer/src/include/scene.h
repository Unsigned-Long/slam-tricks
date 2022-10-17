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
#include "functional"

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
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr _features;

        CubePlane(float roll, float pitch, float yaw,
                  float dx, float dy, float dz,
                  float width, float height = 4.5f, float thickness = 0.1f,
                  int featureNum = 5, const Colour &color = CubePlane::_colourWheel.GetUniqueColour());

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
            // shot
            using std::placeholders::_1;
            _viewer->registerKeyboardCallback(std::bind(&Scene::KeyBoardCallBack, this, _1));

            _viewer->setCameraPosition(-3.0f, 0.0f, 20.0f, -3.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f);
            // _viewer->setCameraPosition(3.0f, 0.0f, 0.0f, -3.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
            // _viewer->setCameraPosition(3.0f, 0.0f, 3.0f, -3.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f);
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

        void KeyBoardCallBack(const pcl::visualization::KeyboardEvent &ev);
    };
}


#endif //ST16_PCL_VIEWER_SCENE_H
