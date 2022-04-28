/**
 * @file undistort_pt.hpp
 * @author shlchen (3079625093@qq.com)
 * @brief to undistort a pixel point
 * @version 0.1
 * @date 2022-04-28
 * 
 * @copyright Copyright (c) 2022
 */
#ifndef UNDISTORT_PT_H
#define UNDISTORT_PT_H

#include "opencv2/core.hpp"
#include "struct_define.hpp"

namespace ns_st1 {
  /**
   * @brief undistort a pixel point
   *
   * @param srcPt the distorted point
   * @param innerParam the camera's inner parameters
   * @param distCoff the camera's distortion parameters
   * @param threshold the threshold to stop iterator
   * @param iterator the interator times
   * @return cv::Point2f the undistoerted pixel point
   */
  static cv::Point2f undistortPoint(
      cv::Point2f srcPt,
      const CameraInnerParam &innerParam,
      const CameraDistCoff &distCoff,
      float threshold = 1E-5,
      int iterator = 5) {
    // check input parameters
    CV_Assert(threshold > 0.0f);
    CV_Assert(iterator > 0);

    // get parameters
    float fx = innerParam.fx, fy = innerParam.fy, cx = innerParam.cx, cy = innerParam.cy;
    float k1 = distCoff.k1, k2 = distCoff.k2, k3 = distCoff.k3;
    float p1 = distCoff.p1, p2 = distCoff.p2;
    float x_est = srcPt.x, y_est = srcPt.y;

    for (int i = 0; i != iterator; ++i) {
      // transform to normalized pixel coordinate plane
      float x = (x_est - cx) / fx;
      float y = (y_est - cy) / fy;
      float r2 = x * x + y * y, r4 = r2 * r2, r6 = r4 * r2;

      // compute the jacobian matrix
      float Juu = -(1.0f + k1 * r2 + k2 * r4 + k3 * r6) -
                  x * (2.0f * k1 * x + 4.0f * k2 * r2 * x + 6.0f * k3 * r4 * x) -
                  2.0f * p1 * y - 6.0f * p2 * x;

      float Jvv = -(1.0f + k1 * r2 + k2 * r4 + k3 * r6) -
                  y * (2.0f * k1 * y + 4.0f * k2 * r2 * y + 6.0f * k3 * r4 * y) -
                  2.0f * p2 * x - 6.0f * p1 * y;

      // the distortion model
      float x_dist = x * (1 + k1 * r2 + k2 * r4 + k3 * r6) + 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
      float y_dist = y * (1 + k1 * r2 + k2 * r4 + k3 * r6) + 2 * p2 * x * y + p1 * (r2 + 2 * y * y);

      // transfrom to pixel coordinate plane
      float u_dist = x_dist * fx + cx;
      float v_dist = y_dist * fy + cy;

      // comput the error
      float eu = srcPt.x - u_dist;
      float ev = srcPt.y - v_dist;

      // the variation
      float delte_u = -eu / Juu;
      float delte_v = -ev / Jvv;

      // update
      x_est += delte_u;
      y_est += delte_v;

      float variation = delte_u * delte_u + delte_v * delte_v;

      if (variation < threshold) {
        break;
      }
    }

    return cv::Point2f(x_est, y_est);
  }
} // namespace ns_st1

#endif