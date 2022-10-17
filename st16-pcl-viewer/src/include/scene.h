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

        CubePlane(Posed pose, float xSpan, float ySpan, float zSpan,
                  const Colour &color = CubePlane::_colourWheel.GetUniqueColour())
                : _pose(std::move(pose)), _xSpan(xSpan), _ySpan(ySpan), _zSpan(zSpan), _color(color) {}
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
