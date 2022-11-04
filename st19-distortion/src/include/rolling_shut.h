//
// Created by csl on 11/4/22.
//

#ifndef ST19_DISTORTION_ROLLING_SHUT_H
#define ST19_DISTORTION_ROLLING_SHUT_H

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

    class CameraScene {
    private:
        cv::Mat _scene;
        std::shared_ptr<std::thread> _displayThread;
        std::shared_ptr<std::thread> _buildThread;

        bool _finished;
        double _factor;

    public:

        static cv::Scalar Colour2Scalar(const Colour &colour) {
            return {colour.b * 255.0, colour.g * 255.0, colour.r * 255.0};
        }

        CameraScene()
                : _scene(640, 640, CV_8UC3, cv::Scalar(100, 100, 0)),
                  _displayThread(nullptr), _buildThread(nullptr), _finished(false), _factor(0.0) {}

        void RunInMultiThread() {
            _displayThread = std::make_shared<std::thread>([this]() {
                const auto center = cv::Point(_scene.cols / 2, _scene.rows / 2);
                const auto radius = 120;
                const auto boxSize = 50;
                _buildThread = std::make_shared<std::thread>([this, &center, &radius, &boxSize]() {
                    while (!_finished) {
                        ColourWheel colourWheel;

                        _scene = cv::Mat(640, 640, CV_8UC3, Colour2Scalar(Colour::White()));
                        _factor += 0.005;
                        auto x = static_cast<int>(radius * std::cos(_factor));
                        auto y = static_cast<int>(radius * std::sin(_factor));

                        auto point = cv::Point(x, y) + center;
                        cv::circle(_scene, point, 50, Colour2Scalar(colourWheel.GetUniqueColour()), cv::FILLED);
                        cv::circle(_scene, center, 20, Colour2Scalar(colourWheel.GetUniqueColour()), cv::FILLED);

                        cv::rectangle(
                                _scene, cv::Point(x + center.x, boxSize),
                                cv::Point(x + center.x + boxSize, boxSize * 2),
                                Colour2Scalar(colourWheel.GetUniqueColour()), cv::FILLED
                        );

                        cv::rectangle(
                                _scene, cv::Point(y + center.x, _scene.rows - boxSize * 2),
                                cv::Point(y + center.x + boxSize, _scene.rows - boxSize),
                                Colour2Scalar(colourWheel.GetUniqueColour()), cv::FILLED
                        );

                        cv::rectangle(
                                _scene, cv::Point(boxSize, x + center.x),
                                cv::Point(boxSize * 2, x + center.x + boxSize),
                                Colour2Scalar(colourWheel.GetUniqueColour()), cv::FILLED
                        );

                        cv::rectangle(
                                _scene, cv::Point(_scene.cols - boxSize * 2, y + center.x),
                                cv::Point(_scene.cols - boxSize, y + center.x + boxSize),
                                Colour2Scalar(colourWheel.GetUniqueColour()), cv::FILLED
                        );


                    }
                });
            });
        }

        void SetFinished() {
            _finished = true;
        }

        virtual ~CameraScene() {
            _buildThread->join();
            _displayThread->join();
        }

    protected:
        void Initialization() {
        }
    };
}

#endif //ST19_DISTORTION_ROLLING_SHUT_H
