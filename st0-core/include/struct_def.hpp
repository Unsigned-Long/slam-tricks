/**
 * @file struct_define.hpp
 * @author shlchen (3079625093@qq.com)
 * @brief define the parameter packages
 * @version 0.1
 * @date 2022-04-28
 *
 * @copyright Copyright (c) 2022
 */
#ifndef STRUCT_DEFINE_H
#define STRUCT_DEFINE_H

namespace ns_st0 {
  /**
   * @brief a structure for camera' inner parameters' organization
   */
  struct CameraInnerParam {
    float fx, fy, cx, cy;
    CameraInnerParam(const float &fx, const float &fy, const float &cx, const float &cy)
        : fx(fx), fy(fy), cx(cx), cy(cy) {}
  };

  /**
   * @brief a structure for camera' distortion parameters' organization
   */
  struct CameraDistCoeff {
    float k1, k2, k3;
    float p1, p2;

    CameraDistCoeff(const float &k1, const float &k2, const float &k3,
                    const float &p1, const float &p2)
        : k1(k1), k2(k2), k3(k3), p1(p1), p2(p2) {}
  };

  /**
   * @brief the different interpolation choices
   */
  enum class Interpolation {
    NEAREST_NEIGHBOR,
    BILINEAR
  };
}

#endif