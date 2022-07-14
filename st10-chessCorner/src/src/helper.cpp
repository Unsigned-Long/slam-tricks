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

  std::vector<cv::Point2i> nms2d(const cv::Mat img, const ushort hws) {
    std::vector<cv::Point2i> max;
    int rows = img.rows, cols = img.cols;
    ushort ws = 2 * hws + 1;
    for (int i = 0; i < rows; i += hws + 1) {
      for (int j = 0; j < cols; j += hws + 1) {
        // find max val and idx
        float maxVal;
        cv::Point2i maxIdx;
        {
          cv::Mat win = img(cv::Range(i, std::min(i + hws + 1, rows)), cv::Range(j, std::min(j + hws + 1, cols)));
          double maxVal_t;
          int maxIdx_t[2];
          cv::minMaxIdx(win, nullptr, &maxVal_t, nullptr, maxIdx_t);
          maxVal = maxVal_t;
          maxIdx.y = maxIdx_t[0] + i;
          maxIdx.x = maxIdx_t[1] + j;
        }

        bool isMax = true;
        for (int r = std::max(0, maxIdx.y - hws); r < std::min(maxIdx.y + hws + 1, rows); ++r) {
          for (int c = std::max(0, maxIdx.x - hws); c < std::min(maxIdx.x + hws + 1, cols); ++c) {
            if (r == maxIdx.y && c == maxIdx.x) {
              continue;
            }
            float val = img.at<float>(r, c);
            if (val > maxVal) {
              isMax = false;
              break;
            }
          }
          if (!isMax) {
            break;
          }
        }
        if (isMax) {
          max.push_back(maxIdx);
        }
      }
    }
    return max;
  }

  cv::Mat drawMarks(cv::Mat grayImg, const std::vector<cv::Point> &pts) {
    cv::Mat color;
    cv::cvtColor(grayImg, color, cv::COLOR_GRAY2BGR);
    for (const auto &pt : pts) {
      cv::drawMarker(color, pt, cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 10);
    }
    return color;
  }

} // namespace ns_st10
