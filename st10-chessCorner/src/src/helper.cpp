#include "helper.h"
#include <cmath>

namespace ns_st10 {
  cv::Mat cvt_32FC1_8UC1(cv::Mat img_f) {
    double min, max;
    cv::minMaxIdx(img_f, &min, &max);
    cv::Mat gray(img_f.size(), CV_8UC1);
    // 255*(pixel - min)/(max - min)
    float factor = 255.0 / (max - min);
    img_f.convertTo(gray, CV_8UC1, factor, -min * factor);
    return gray;
  }

  float gauss(float x, float mean, float sigma) {
    const float sqrt_2pi = std::sqrt(2.0f * M_PI);
    const float index = -(x - mean) * (x - mean) / (2.0f * sigma * sigma);
    return 1.0 / (sqrt_2pi * sigma) * std::exp(index);
  }

  void showImg(cv::Mat img, const std::string &name) {
    cv::namedWindow(name, cv::WINDOW_FREERATIO);
    cv::imshow(name, img);
    return;
  }

  bool inAngleRange(float angle, float startAng, float endAng) {
    if (angle < 0.0f) {
      angle += 2.0 * M_PI;
    } else if (angle > 2.0 * M_PI) {
      angle -= 2.0 * M_PI;
    }
    if (startAng < 0.0f) {
      startAng += 2.0 * M_PI;
    } else if (startAng > 2.0 * M_PI) {
      startAng -= 2.0 * M_PI;
    }
    if (endAng < 0.0f) {
      endAng += 2.0 * M_PI;
    } else if (endAng > 2.0 * M_PI) {
      endAng -= 2.0 * M_PI;
    }
    if (startAng < endAng) {
      return angle > startAng && angle < endAng;
    } else {
      return angle > startAng || angle < endAng;
    }
  }
} // namespace ns_st10
