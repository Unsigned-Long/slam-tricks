/**
 * @file undistort_img.hpp
 * @author shlchen (3079625093@qq.com)
 * @brief to undistort a distorted image
 * @version 0.1
 * @date 2022-04-28
 *
 * @copyright Copyright (c) 2022
 */
#ifndef UNDISTORT_IMG_H
#define UNDISTORT_IMG_H

#include "opencv2/core.hpp"
#include "struct_def.hpp"
#include <functional>
#include <iostream>

using namespace ns_st0;

namespace ns_st1 {
  /**
   * @brief undistort a gray image
   *
   * @param src the distorted gray image [CV_8UC1]
   * @param innerParam the camera's inner parameters
   * @param distCoff the camera's distortion parameters
   * @param methods the interpolation choice
   * @return cv::Mat the undistorted gray image [CV_8UC1]
   */
  static cv::Mat undistortImage(
      cv::Mat src,
      const CameraInnerParam &innerParam,
      const CameraDistCoeff &distCoff,
      Interpolation methods = Interpolation::NEAREST_NEIGHBOR) {
    // assert
    CV_Assert(src.type() == CV_8UC1);
    CV_Assert(!src.empty());

    // get parameters
    int rows = src.rows, cols = src.cols;
    float fx = innerParam.fx, fy = innerParam.fy, cx = innerParam.cx, cy = innerParam.cy;
    float k1 = distCoff.k1, k2 = distCoff.k2, k3 = distCoff.k3;
    float p1 = distCoff.p1, p2 = distCoff.p2;

    // define the interpolation function
    std::function<char((float, float, uchar *))> interpolation;

    switch (methods) {
    case Interpolation::NEAREST_NEIGHBOR: {
      interpolation = [cols, rows](float u, float v, uchar *headPtr) {
        return *(headPtr + int(v + 0.5) * cols + int(u + 0.5));
      };
    } break;
    case Interpolation::BILINEAR: {
      interpolation = [cols, rows](float u, float v, uchar *headPtr) {
        // four corners
        int u_lt = int(u), v_lt = int(v);
        int u_rt = u_lt + 1, v_rt = v_lt;
        int u_lb = u_lt, v_lb = v_lt + 1;
        int u_rb = u_lt + 1, v_rb = v_lt + 1;

        // range check for u
        (u_lt < 0) ? (u_lt = 0) : (0), (u_lt > cols - 1) ? (u_lt = cols - 1) : (0);
        (u_rt < 0) ? (u_rt = 0) : (0), (u_rt > cols - 1) ? (u_rt = cols - 1) : (0);
        (u_lb < 0) ? (u_lb = 0) : (0), (u_lb > cols - 1) ? (u_lb = cols - 1) : (0);
        (u_rb < 0) ? (u_rb = 0) : (0), (u_rb > cols - 1) ? (u_rb = cols - 1) : (0);

        // range check for v
        (v_lt < 0) ? (v_lt = 0) : (0), (v_lt > rows - 1) ? (v_lt = rows - 1) : (0);
        (v_rt < 0) ? (v_rt = 0) : (0), (v_rt > rows - 1) ? (v_rt = rows - 1) : (0);
        (v_lb < 0) ? (v_lb = 0) : (0), (v_lb > rows - 1) ? (v_lb = rows - 1) : (0);
        (v_rb < 0) ? (v_rb = 0) : (0), (v_rb > rows - 1) ? (v_rb = rows - 1) : (0);

        // the gray values for four corners
        uchar lt = *(headPtr + v_lt * cols + u_lt);
        uchar rt = *(headPtr + v_rt * cols + u_rt);
        uchar lb = *(headPtr + v_lb * cols + u_lb);
        uchar rb = *(headPtr + v_rb * cols + u_rb);

        // bilinear
        float v1 = (u - int(u)) * rt + (1 - u + int(u)) * lt;
        float v2 = (u - int(u)) * rb + (1 - u + int(u)) * lb;

        uchar v3 = (v - int(v)) * v2 + (1 - v + int(v)) * v1 + 0.5;

        return v3;
      };
    } break;
    default: {
      interpolation = [cols, rows](float u, float v, uchar *headPtr) {
        if (u < 0 || u > cols - 1 || v < 0 || v > rows - 1) {
          return uchar(0);
        } else {
          return *(headPtr + int(v) * cols + int(u));
        }
      };
    } break;
    }

    // the undistorted image
    cv::Mat dst(rows, cols, CV_8UC1);
    uchar *srcHead = src.ptr<uchar>(0);

    for (int v = 0; v != rows; ++v) {
      auto dstPtr = dst.ptr<uchar>(v);

      for (int u = 0; u != cols; ++u) {

        // transfrom to the normalized pixel coordinate
        float x = (u - cx) / fx;
        float y = (v - cy) / fy;
        float r2 = x * x + y * y, r4 = r2 * r2, r6 = r4 * r2;

        // the distortion model
        float x_dist = x * (1 + k1 * r2 + k2 * r4 + k3 * r6) + 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
        float y_dist = y * (1 + k1 * r2 + k2 * r4 + k3 * r6) + 2 * p2 * x * y + p1 * (r2 + 2 * y * y);

        // transfrom to pixel coordinate
        float u_dist = x_dist * fx + cx;
        float v_dist = y_dist * fy + cy;

        // range check
        if (u_dist < 0 || u_dist > cols - 1 || v_dist < 0 || v_dist > rows - 1) {
          dstPtr[u] = 0;
        } else {
          // interpolation
          dstPtr[u] = interpolation(u_dist, v_dist, srcHead);
        }
      }
    }

    return dst;
  }
} // namespace ns_st1

#endif