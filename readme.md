# Slam Tricks

*__Author: shlChen__*

***E-Mail: 3079625093@qq.com***

[TOC]

## 1. Overview

本项目旨在记录一些SLAM中常用到的一些算法和小技巧，有些能够直接改善代码的效率，有些则能够给人一些启发。目前实现了一些简单、有用的小算法。

## 2. Details

### 2.1 [图像和像素点去畸变](./st1-undistort/) 

SLAM中一般不会谈及图像畸变的问题，因为我们都假设图像或者像素点都已经基于畸变模型和畸变参数做了处理，传入的是一些不存在畸变的像片或者像素点。但是这确实一个很必要优化的东西，虽然OpenCV库已经提供了比较完整的函数、方法。该项目的原理介绍以PDF的格式展示，具体的位置为[算法原理](./st1-undistort/docs/undistort.pdf)。项目的成果：

+ 完成了基于图片的去畸变算法。在算法中使用了两种插值算法（最邻近和双线性）：

```cpp
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
      const CameraDistCoff &distCoff,
      Interpolation methods = Interpolation::NEAREST_NEIGHBOR) {...}
```



+ 完成了基于点的去畸变算法。其依托于高斯牛顿法，速度和精度都比OpenCV提供的要好：

```cpp
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
      int iterator = 5) {...}
```

### 2.2 [对极几何](./st2-epipolar)

在使用单目相机进行$SLAM$的时候，必不可少的一步就是初始化。由于单目相机的尺度是未知的，所以一般初始化的方式是通过对极几何约束，解算出两帧图像之间的位姿变换，来进行初始化。该项目的原理介绍以PDF的格式展示，具体的位置为[算法原理](./st2-epipolar/docs/epipolar.pdf)。项目的成果：

+ 实现了基于卡方检验的本质矩阵求解：

```cpp
  /**
   * @brief to get function matrix based on the epipolar constraints
   *
   * @param kps1 the keypoints in the first image
   * @param kps2 the keypoints in the second image
   * @param srcMatches the source matches, It can be matching data without preprocessing
   * @param goodMatches the good matches that this algorithm return
   * @param quantile the quantile to judge whether a match is an outlier
   * @return Eigen::Matrix3f the function matrix
   */
  static Eigen::Matrix3f solveEpipolar(
      const std::vector<cv::KeyPoint> &kps1,
      const std::vector<cv::KeyPoint> &kps2,
      const std::vector<cv::DMatch> &srcMatches,
      const CameraInnerParam &innerParam,
      std::vector<cv::DMatch> *goodMatches = nullptr,
      float quantile = 1.323) {...}
```



+ 实现了基于本质矩阵的运动恢复：

```cpp
  /**
   * @brief recovery the movement from the essential matrix
   *
   * @param eMatrix the essential matrix
   * @param K the camera's inner paramters
   * @param kp1 the key point in first frame
   * @param kp2 the key point in second frame
   * @param rot21 rotation matrix from first frame to second frame
   * @param t21 translation matrix from first frame to second frame
   * @return true the process is successful
   * @return false the process is failed
   */
  static bool recoveryMove(
      const Eigen::Matrix3f &eMatrix,
      const Eigen::Matrix3f &K,
      const cv::KeyPoint &kp1,
      const cv::KeyPoint &kp2,
      Eigen::Matrix3f &rot21,
      Eigen::Vector3f &t21) {...}
```



+ 实现了三角化：

```cpp
  /**
   * @brief to trangular a pair points on the normalized coordinate
   *
   * @param X1 the point on the first camera's normalized coordinate
   * @param X2 the point on the second camera's normalized coordinate
   * @param rot21 the rotation from first camera to second camera
   * @param t21 the translation from first camera to second camera
   * @param P1 the point on the first camera's coordinate
   * @param P2 the point on the second camera's coordinate
   * @return std::pair<float, float> the depth pair
   */
  static std::pair<float, float> triangulation(
      const Eigen::Vector3f &X1,
      const Eigen::Vector3f &X2,
      const Eigen::Matrix3f &rot21,
      const Eigen::Vector3f &t21,
      Eigen::Vector3f *P1 = nullptr,
      Eigen::Vector3f *P2 = nullptr) {...}
```

