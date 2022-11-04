//
// Created by csl on 11/4/22.
//

#ifndef ST19_DISTORTION_LIDAR_H
#define ST19_DISTORTION_LIDAR_H

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "artwork/logger/logger.h"
#include "colour.hpp"

namespace ns_st19 {

    class LidarScene {
    private:
        bool _finished;
        double _factor;

        const int IMG_SIZE = 640;
        const int PADDING_SIZE = 120;
        const int LINE_SIZE = 2;
        const int LIDAR_SIZE = 30;

        double _speed;

        cv::Mat _shut, _fullScene;
        const std::string _dirname;

        std::vector<std::pair<cv::Point, cv::Point>> _walls;

        cv::Point _lidarPos;

        double _curDeg;
    public:

        static cv::Scalar Colour2Scalar(const Colour &colour) {
            return {colour.b * 255.0, colour.g * 255.0, colour.r * 255.0};
        }

        explicit LidarScene(std::string dirname)
                : _finished(false), _factor(0.0), _lidarPos(IMG_SIZE / 2, IMG_SIZE / 2),
                  _shut(IMG_SIZE, IMG_SIZE, CV_8UC3, cv::Scalar(255, 255, 255)), _curDeg(0),
                  _dirname(std::move(dirname)), _speed(0.005) {
            _walls = {
                    {
                            {PADDING_SIZE,            PADDING_SIZE},
                            {IMG_SIZE - PADDING_SIZE, PADDING_SIZE}
                    },
                    {       {PADDING_SIZE,            IMG_SIZE - PADDING_SIZE},
                            {IMG_SIZE - PADDING_SIZE, IMG_SIZE - PADDING_SIZE}
                    },
                    {       {PADDING_SIZE,            PADDING_SIZE},
                            {PADDING_SIZE,            IMG_SIZE - PADDING_SIZE}
                    },
                    {       {IMG_SIZE - PADDING_SIZE, PADDING_SIZE},
                            {IMG_SIZE - PADDING_SIZE, IMG_SIZE - PADDING_SIZE}
                    }
            };
        }

        void Run() {
            InitializeWindow();

            while (!_finished) {
                auto scene = GenerateScene();

                const static double DEG_2_RAD = M_PI / 180.0;
                const static cv::Point CENTER = cv::Point(IMG_SIZE / 2, IMG_SIZE / 2);

                double radian = _curDeg * DEG_2_RAD;
                cv::Point tarPoint = _lidarPos;
                tarPoint.y += IMG_SIZE * std::cos(radian);
                tarPoint.x += IMG_SIZE * std::sin(radian);

                for (const auto &[p1, p2]: _walls) {
                    if (auto p = LineIntersection(_lidarPos, tarPoint, p1, p2)) {
                        cv::circle(scene, *p, LINE_SIZE * 2, Colour2Scalar(Colour::White()), cv::FILLED);
                        cv::line(scene, _lidarPos, *p, Colour2Scalar(Colour::White()), 1);

                        cv::circle(_shut, *p - _lidarPos + CENTER,
                                   LINE_SIZE, Colour2Scalar(Colour::Blue()), cv::FILLED);

                        cv::line(_shut, *p - _lidarPos + CENTER, CENTER, Colour2Scalar(Colour::Black()), 1);

                        break;
                    }
                }

                _fullScene = cv::Mat(IMG_SIZE, IMG_SIZE * 2, CV_8UC3);
                scene.copyTo(_fullScene(cv::Range(0, IMG_SIZE), cv::Range(0, IMG_SIZE)));
                _shut.copyTo(_fullScene(cv::Range(0, IMG_SIZE), cv::Range(IMG_SIZE, IMG_SIZE * 2)));
                _fullScene.col(IMG_SIZE).setTo(cv::Scalar(0, 0, 0));
                AddOrigin(_shut, CENTER);

                _curDeg += 1.0;
                if (_curDeg > 360.0) {
                    _curDeg = 0.0;
                    _shut = cv::Mat(IMG_SIZE, IMG_SIZE, CV_8UC3, cv::Scalar(255, 255, 255));
                }

                cv::imshow("Lidar-Scene", _fullScene);
                cv::waitKey(5);
            }
        }

        void SetSpeed(double speed) {
            _speed = speed;
        }

    protected:
        cv::Mat GenerateScene() {
            const auto CENTER = cv::Point(IMG_SIZE / 2, IMG_SIZE / 2);

            ColourWheel colourWheel;
            // remove the first two colors
            colourWheel.GetUniqueColour();
            colourWheel.GetUniqueColour();

            auto scene = cv::Mat(IMG_SIZE, IMG_SIZE, CV_8UC3, Colour2Scalar(colourWheel.GetUniqueColour()));
            for (const auto &[p1, p2]: _walls) {
                cv::line(scene, p1, p2, Colour2Scalar(Colour::Black()), LINE_SIZE);
            }
            _factor += _speed;
            auto y = static_cast<int>(100 * std::sin(_factor));
            _lidarPos.x = IMG_SIZE / 2 - y;
            _lidarPos.y = IMG_SIZE / 2;
            cv::circle(scene, _lidarPos, LIDAR_SIZE, Colour2Scalar(colourWheel.GetUniqueColour()), cv::FILLED);
            AddOrigin(scene, _lidarPos);

            return scene;
        }

        void InitializeWindow() {
            cv::namedWindow("Lidar-Scene");
            cv::setMouseCallback("Lidar-Scene", [](int event, int x, int y, int flags, void *param) {
                auto lidarScene = static_cast<LidarScene *>(param);
                if (event == cv::EVENT_MBUTTONDOWN) {
                    lidarScene->_finished = true;
                } else if (event == cv::EVENT_RBUTTONDOWN) {
                    std::int64_t curTimeStamp = std::chrono::system_clock::now().time_since_epoch().count();
                    const std::string filename = lidarScene->_dirname + "/" + std::to_string(curTimeStamp) + ".png";
                    cv::imwrite(filename, lidarScene->_fullScene);
                    LOG_PLAINTEXT("save current scene to path: '", filename, "'.")
                }
            }, this);
            LOG_INFO("press the middle button of the mouse to quit.")
            LOG_INFO("press the right button of the mouse to save the lidar scene.")
        }

        static std::optional<cv::Point> LineIntersection(const cv::Point &l1p1, const cv::Point &l1p2,
                                                         const cv::Point &l2p1, const cv::Point &l2p2) {
            cv::Point pt;
            // line1's component
            double X1 = l1p2.x - l1p1.x;//b1
            double Y1 = l1p2.y - l1p1.y;//a1
            // line2's component
            double X2 = l2p2.x - l2p1.x;//b2
            double Y2 = l2p2.y - l2p1.y;//a2
            // distance of 1,2
            double X21 = l2p1.x - l1p1.x;
            double Y21 = l2p1.y - l1p1.y;
            // determinant
            double D = Y1 * X2 - Y2 * X1;// a1b2-a2b1
            // 
            if (D == 0) {
                return {};
            }
            // cross point
            pt.x = static_cast<int>((X1 * X2 * Y21 + Y1 * X2 * l1p1.x - Y2 * X1 * l2p1.x) / D);
            // on screen y is down increased ! 
            pt.y = static_cast<int>(-(Y1 * Y2 * X21 + X1 * Y2 * l1p1.y - X2 * Y1 * l2p1.y) / D);
            // segments intersect.
            if ((abs(pt.x - l1p1.x - X1 / 2) <= abs(X1 / 2)) &&
                (abs(pt.y - l1p1.y - Y1 / 2) <= abs(Y1 / 2)) &&
                (abs(pt.x - l2p1.x - X2 / 2) <= abs(X2 / 2)) &&
                (abs(pt.y - l2p1.y - Y2 / 2) <= abs(Y2 / 2))) {
                return pt;
            }
            return {};
        }

        void AddOrigin(cv::Mat img, const cv::Point &origin) const {
            // x
            cv::line(
                    img, origin, cv::Point(origin.x, origin.y + LIDAR_SIZE),
                    Colour2Scalar(Colour::Red()), LINE_SIZE
            );
            // y
            cv::line(
                    img, origin, cv::Point(origin.x + LIDAR_SIZE, origin.y),
                    Colour2Scalar(Colour::Green()), LINE_SIZE
            );
            // origin
            cv::circle(img, origin, LINE_SIZE * 2, Colour2Scalar(Colour::Blue()), cv::FILLED);
        }
    };
}

#endif //ST19_DISTORTION_LIDAR_H
