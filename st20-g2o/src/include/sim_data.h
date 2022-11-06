//
// Created by csl on 11/5/22.
//

#ifndef ST20_G2O_SIM_DATA_H
#define ST20_G2O_SIM_DATA_H

#include "slam-scene-viewer/scene_viewer.h"

namespace ns_st20 {

    class ProblemScene : public ns_viewer::SceneViewer {
    public:
        using parent_type = ns_viewer::SceneViewer;

    private:
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr _features;
        // feature -> cameras
        std::vector<std::vector<std::size_t>> _featureCamerasVec;

        std::vector<ns_viewer::Posed> _cameraTraj;
        // camera -> features
        std::vector<std::vector<std::size_t>> _cameraFeatureVec;

        const double CAM_PLANE_HALF_WIDTH = 0.8;
        const double CAM_PLANE_HALF_HEIGHT = 0.6;

    public:
        explicit ProblemScene(std::string sceneShotSaveDir = "");

        void ShowCameraAt(std::size_t cameraIdx = 0);

        void ShowFeatureAt(std::size_t featureIdx = 0);

        void ShowCameras();

        void ShowFeatures();

        ~ProblemScene() override;

    protected:

        void CreateTrajectory();

        void CreateScene();

        void CreateMeasurements();
    };
}


#endif //ST20_G2O_SIM_DATA_H
