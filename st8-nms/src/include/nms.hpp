#ifndef NMS_HPP
#define NMS_HPP

#define FORMAT_VECTOR
#include "artwork/logger/logger.h"
#include "opencv2/opencv.hpp"
#include <cmath>

namespace ns_st8 {
  static std::vector<ushort> nms1d(const std::vector<float> &vals, const ushort hws) {
    std::vector<ushort> max;
    for (int i = hws; i < vals.size() - 2 * hws;) {
      float maxVal = vals[i];
      ushort maxIdx = i;
      for (int j = i; j < i + hws + 1; ++j) {
        if (vals[j] > vals[i]) {
          maxIdx = j;
          maxVal = vals[j];
        }
      }
      bool findMax = true, hasLower = false;
      for (int k = maxIdx - hws; k < maxIdx + hws + 1; ++k) {
        if (k == maxIdx) {
          continue;
        }
        if (vals[k] > maxVal) {
          findMax = false;
          break;
        } else if (vals[k] < maxVal) {
          hasLower = true;
        }
      }
      if (findMax && hasLower) {
        max.push_back(maxIdx);
        i = maxIdx + hws + 1;
      } else {
        i += 1;
      }
    }
    return max;
  }
  static std::vector<cv::Point2i> nms2d(const cv::Mat img, const ushort hws) {
    std::vector<cv::Point2i> max;
    int rows = img.rows, cols = img.cols;
    for (int i = hws; i < rows - 2 * hws; i += hws + 1) {
      for (int j = hws; j < cols - 2 * hws; j += hws + 1) {
        uchar maxVal = img.at<uchar>(i, j);
        cv::Point2i maxIdx(j, i);
        for (int r = i; r < i + hws + 1; ++r) {
          for (int c = j; c < j + hws + 1; ++c) {
            uchar val = img.at<uchar>(r, c);
            if (val > maxVal) {
              maxVal = val;
              maxIdx = cv::Point2i(c, r);
            }
          }
        }
        bool findMax = true, hasLower = false;
        for (int r = maxIdx.y - hws; r < maxIdx.y + hws + 1; ++r) {
          for (int c = maxIdx.x - hws; c < maxIdx.x + hws + 1; ++c) {
            if (r == maxIdx.y && c == maxIdx.x) {
              continue;
            }
            uchar val = img.at<uchar>(r, c);
            if (val > maxVal) {
              findMax = false;
              break;
            } else if (val < maxVal) {
              hasLower = true;
            }
          }
        }
        if (findMax && hasLower) {
          max.push_back(maxIdx);
        }
      }
    }
    return max;
  }
} // namespace ns_st7

#endif