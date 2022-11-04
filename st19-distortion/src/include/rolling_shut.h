//
// Created by csl on 11/4/22.
//

#ifndef ST19_DISTORTION_ROLLING_SHUT_H
#define ST19_DISTORTION_ROLLING_SHUT_H

#include <utility>

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "thread"
#include "memory"
#include "artwork/logger/logger.h"
#include "artwork/timer/timer.h"
#include "mutex"
#include "colour.hpp"

namespace ns_st19 {

    std::mutex mt;

    class CameraScene {
    private:
        std::shared_ptr<std::thread> _buildThread;

        bool _finished;
        double _factor;

        const int IMG_SIZE = 640;
        int _curRowIdx;
        int _shutSpeedInv;

        cv::Mat _shut, _fullScene;

        const std::string _dirname;
    public:

        static cv::Scalar Colour2Scalar(const Colour &colour) {
            return {colour.b * 255.0, colour.g * 255.0, colour.r * 255.0};
        }

        explicit CameraScene(std::string dirname)
                : _buildThread(nullptr), _finished(false), _factor(0.0), _shutSpeedInv(1),
                  _curRowIdx(0), _shut(IMG_SIZE, IMG_SIZE, CV_8UC3, cv::Scalar(255, 255, 255)),
                  _dirname(std::move(dirname)) {}

        void Run() {
            InitializeWindow();

            while (!_finished) {
                auto scene = GenerateScene();
                scene.row(_curRowIdx / _shutSpeedInv).copyTo(_shut.row(_curRowIdx / _shutSpeedInv));

                _fullScene = cv::Mat(IMG_SIZE, IMG_SIZE * 2, CV_8UC3);
                scene.copyTo(_fullScene(cv::Range(0, IMG_SIZE), cv::Range(0, IMG_SIZE)));
                _shut.copyTo(_fullScene(cv::Range(0, IMG_SIZE), cv::Range(IMG_SIZE, IMG_SIZE * 2)));

                _fullScene.col(IMG_SIZE).setTo(cv::Scalar(0, 0, 0));

                cv::imshow("Camera-Rolling-Shut", _fullScene);

                ++_curRowIdx;
                if (_curRowIdx / _shutSpeedInv >= IMG_SIZE) {
                    _curRowIdx = 0;
                    _shut = cv::Mat(IMG_SIZE, IMG_SIZE, CV_8UC3, cv::Scalar(255, 255, 255));
                }
                cv::waitKey(5);
            }
        }

        void SetFinished() {
            _finished = true;
        }

        void SetShutSpeedInv(int shutSpeedInv) {
            _shutSpeedInv = shutSpeedInv;
        }

        virtual ~CameraScene() {
            _buildThread->join();
        }

    protected:
        cv::Mat GenerateScene() {

            const auto CENTER = cv::Point(IMG_SIZE / 2, IMG_SIZE / 2);
            const auto RADIUS = 120;
            const auto BOX_SIZE = 60;

            ColourWheel colourWheel;
            // remove the first two colors
            colourWheel.GetUniqueColour();
            colourWheel.GetUniqueColour();

            auto scene = cv::Mat(IMG_SIZE, IMG_SIZE, CV_8UC3, Colour2Scalar(colourWheel.GetUniqueColour()));
            _factor += 0.005;
            auto x = static_cast<int>(RADIUS * std::cos(_factor));
            auto y = static_cast<int>(RADIUS * std::sin(_factor));

            auto point = cv::Point(x, y) + CENTER;

            cv::circle(scene, point, 50, Colour2Scalar(colourWheel.GetUniqueColour()), cv::FILLED);
            {
                auto x1 = static_cast<int>(RADIUS * std::cos(_factor + M_PI * 0.67));
                auto y1 = static_cast<int>(RADIUS * std::sin(_factor + M_PI * 0.67));
                auto point1 = cv::Point(x1, y1) + CENTER;
                cv::circle(scene, point1, 50, Colour2Scalar(colourWheel.GetUniqueColour()), cv::FILLED);

                auto x2 = static_cast<int>(RADIUS * std::cos(_factor + M_PI * 1.34));
                auto y2 = static_cast<int>(RADIUS * std::sin(_factor + M_PI * 1.34));
                auto point2 = cv::Point(x2, y2) + CENTER;
                cv::circle(scene, point2, 50, Colour2Scalar(colourWheel.GetUniqueColour()), cv::FILLED);
            }

            cv::circle(scene, CENTER, 50, Colour2Scalar(colourWheel.GetUniqueColour()), cv::FILLED);

            cv::rectangle(
                    scene, cv::Point(x + CENTER.x, BOX_SIZE),
                    cv::Point(x + CENTER.x + BOX_SIZE, BOX_SIZE * 2),
                    Colour2Scalar(colourWheel.GetUniqueColour()), cv::FILLED
            );

            cv::rectangle(
                    scene, cv::Point(y + CENTER.x, IMG_SIZE - BOX_SIZE * 2),
                    cv::Point(y + CENTER.x + BOX_SIZE, IMG_SIZE - BOX_SIZE),
                    Colour2Scalar(colourWheel.GetUniqueColour()), cv::FILLED
            );

            cv::rectangle(
                    scene, cv::Point(BOX_SIZE, x + CENTER.x),
                    cv::Point(BOX_SIZE * 2, x + CENTER.x + BOX_SIZE),
                    Colour2Scalar(colourWheel.GetUniqueColour()), cv::FILLED
            );

            cv::rectangle(
                    scene, cv::Point(IMG_SIZE - BOX_SIZE * 2, y + CENTER.x),
                    cv::Point(IMG_SIZE - BOX_SIZE, y + CENTER.x + BOX_SIZE),
                    Colour2Scalar(colourWheel.GetUniqueColour()), cv::FILLED
            );

            return scene;
        }

        void InitializeWindow() {
            cv::namedWindow("Camera-Rolling-Shut");
            cv::setMouseCallback("Camera-Rolling-Shut", [](int event, int x, int y, int flags, void *param) {
                auto cameraScene = static_cast<CameraScene *>(param);
                if (event == cv::EVENT_MBUTTONDOWN) {
                    cameraScene->_finished = true;
                } else if (event == cv::EVENT_RBUTTONDOWN) {
                    std::int64_t curTimeStamp = std::chrono::system_clock::now().time_since_epoch().count();
                    const std::string filename = cameraScene->_dirname + "/" + std::to_string(curTimeStamp) + ".png";
                    cv::imwrite(filename, cameraScene->_fullScene);
                    LOG_PLAINTEXT("save current scene to path: '", filename, "'.")
                }
            }, this);
            LOG_INFO("press the middle button of the mouse to quit.")
            LOG_INFO("press the right button of the mouse to save the camera scene.")
        }

    };
}

#endif //ST19_DISTORTION_ROLLING_SHUT_H
