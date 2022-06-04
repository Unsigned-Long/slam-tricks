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

#include "eigen3/Eigen/Dense"
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include <filesystem>

namespace ns_st0 {
  /**
   * @brief a structure for camera' inner parameters' organization
   */
  struct CameraInnerParam {
    float fx, fy, cx, cy;
    CameraInnerParam(const float &fx, const float &fy, const float &cx, const float &cy)
        : fx(fx), fy(fy), cx(cx), cy(cy) {}

    /**
     * @brief organize the parameters to an Eigen matrix
     *
     * @tparam Type the vaule type e.t. double, float...
     * @return Eigen::Matrix<Type, 3, 3> the K matrix
     */
    template <typename Type>
    inline Eigen::Matrix<Type, 3, 3> toEigenMatrix() const {
      Eigen::Matrix<Type, 3, 3> K = Eigen::Matrix<Type, 3, 3>::Identity();
      K(0, 0) = fx, K(1, 1) = fy;
      K(0, 2) = cx, K(1, 2) = cy;
      return K;
    }

    /**
     * @brief organize the parameters to an OpenCV matrix
     *
     * @tparam Type the vaule type e.t. double, float...
     * @return cv::Mat the K matrix
     */
    template <typename Type>
    inline cv::Mat toOpenCVMatrix() const {
      cv::Mat mat = (cv::Mat_<Type>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
      return mat;
    }
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

  /**
   * @brief get antisymmetric matrix from a vector
   *
   * @tparam Type the vaule type
   * @param vec the vector
   * @return Eigen::Matrix<Type, 3, 3> the antisymmetric matrix
   */
  template <typename Type>
  Eigen::Matrix<Type, 3, 3> antisymmetric(const Eigen::Vector<Type, 3> &vec) {
    Type x = vec(0), y = vec(1), z = vec(2);
    Eigen::Matrix<Type, 3, 3> mat;
    mat(0, 0) = 0, mat(0, 1) = -z, mat(0, 2) = y;
    mat(1, 0) = z, mat(1, 1) = 0, mat(1, 2) = -x;
    mat(2, 0) = -y, mat(2, 1) = x, mat(2, 2) = 0;
    return mat;
  }

  /**
   * @brief recovery the vector from a antisymmetric matrix
   *
   * @tparam Type the value type
   * @param mat the antisymmetric matrix
   * @return Eigen::Vector<Type, 3> the vector
   */
  template <typename Type>
  Eigen::Vector<Type, 3> antisymmetric(const Eigen::Matrix<Type, 3, 3> &mat) {
    Eigen::Vector<Type, 3> vec(-mat(1, 2), mat(0, 2), -mat(0, 1));
    return vec;
  }

  /**
   * @brief detect the key points anf match them
   *
   * @param img1 the first image
   * @param img2 the second image
   * @param kps1 the key points in first image
   * @param kps2 the key points in second image
   * @param match the matches
   */
  static void detectAndMatch(cv::Mat img1, cv::Mat img2,
                             std::vector<cv::KeyPoint> &kps1,
                             std::vector<cv::KeyPoint> &kps2,
                             std::vector<cv::DMatch> &match) {
    cv::Mat descriptors1, descriptors2;

    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    // detector
    detector->detect(img1, kps1);
    detector->detect(img2, kps2);

    // compute
    descriptor->compute(img1, kps1, descriptors1);
    descriptor->compute(img2, kps2, descriptors2);

    // match
    matcher->match(descriptors1, descriptors2, match);
    return;
  }

  /**
   * \brief a function to get all the filenames in the directory
   * \param directory the directory
   * \return the filenames in the directory
   */
  static std::vector<std::string> filesInDir(const std::string &directory) {
    std::vector<std::string> files;
    for (const auto &elem : std::filesystem::directory_iterator(directory))
      if (elem.status().type() != std::filesystem::file_type::directory)
        files.push_back(std::filesystem::canonical(elem.path()).c_str());
    return files;
  }

}

#endif