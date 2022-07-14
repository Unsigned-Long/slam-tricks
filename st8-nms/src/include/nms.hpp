#ifndef NMS_HPP
#define NMS_HPP

#define FORMAT_VECTOR
#include "artwork/logger/logger.h"
#include "opencv2/opencv.hpp"
#include <cmath>

namespace ns_st8 {
  static std::vector<ushort> nms1d(const std::vector<float> &vals, const ushort hws) {
    std::vector<ushort> max;
    ushort ws = 2 * hws + 1;
    for (int i = 0; i < vals.size(); i += hws + 1) {
      float maxVal = vals[i];
      int maxIdx = i;
      for (int j = i + 1; j < std::min(i + hws + 1, int(vals.size())); ++j) {
        if (vals[j] > maxVal) {
          maxVal = vals[j];
          maxIdx = j;
        }
      }
      int sIdx = std::max(maxIdx - hws, 0);
      int eIdx = std::min(maxIdx + hws + 1, int(vals.size()));
      for (int k = sIdx; k < eIdx; ++k) {
        if (k == maxIdx) {
          continue;
        }
        if (vals[k] >= maxVal) {
          maxIdx = -1;
          break;
        }
      }
      if (maxIdx != -1) {
        max.push_back(maxIdx);
      }
    }
    return max;
  }

  static std::vector<cv::Point2i> nms2d(const cv::Mat img, const ushort hws) {
    std::vector<cv::Point2i> max;
    int rows = img.rows, cols = img.cols;
    ushort ws = 2 * hws + 1;
    for (int i = 0; i < rows; i += hws + 1) {
      for (int j = 0; j < cols; j += hws + 1) {
        // find max val and idx
        uchar maxVal;
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
            uchar val = img.at<uchar>(r, c);
            if (val >= maxVal) {
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
} // namespace ns_st7

#endif