# Slam Tricks

*__Author: shlChen__*

***E-Mail: 3079625093@qq.com***

[TOC]

## 1. Overview

本项目旨在记录一些SLAM中常用到的一些算法和小技巧，有些能够直接改善代码的效率，有些则能够给人一些启发。目前实现了一些简单、有用的小算法。

## 2. Details

### 2.1 [图像和像素点去畸变](./st1-undistort/) 

SLAM中一般不会谈及图像畸变的问题，因为我们都假设图像或者像素点都已经基于畸变模型和畸变参数做了处理，传入的是一些不存在畸变的像片或者像素点。但是这确实一个很必要优化的东西，虽然OpenCV库已经提供了比较完整的函数、方法。该项目的原理介绍已PDF的格式展示，具体的位置为[算法原理](./st1-undistort/docs/undistort.pdf)。项目的成果：

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

+ 结果

```cpp
{'undistorted-nearest': 5.18229(MS)}
{'undistorted-bilinear': 8.25476(MS)}
------------
gauss-newton
------------
{'undistort point cost': 0.00903(MS)}
srcPt(distorted): [188.00000, 120.00000]
dstPt(undistorted): [174.34048, 110.19159]
error: 0.000043158372137
------
OpenCV
------
{'undistort point cost': 0.05889(MS)}
srcPt(distorted): [188.00000, 120.00000]
dstPt(undistorted): [174.34122, 110.19208]
error: 0.000723787932657
```

