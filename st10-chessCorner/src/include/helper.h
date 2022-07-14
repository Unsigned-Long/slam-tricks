#ifndef HELPER_H
#define HELPER_H

#define FORMAT_VECTOR
#include "artwork/logger/logger.h"
#include "artwork/timer/timer.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

namespace ns_st10 {
  cv::Mat cvt_32FC1_8UC1(cv::Mat img_f);

  float gauss(float x, float mean, float sigma);

  void showImg(cv::Mat img, const std::string &name = "img");

  bool inAngleRange(float angle, float startAng, float endAng);

  std::vector<cv::Point2i> nms2d(const cv::Mat img, const ushort hws);

  cv::Mat drawMarks(cv::Mat grayImg, const std::vector<cv::Point> &pts);

  void gaussFilter(std::vector<float> &ary);

  std::pair<std::size_t, std::size_t> meanShift(const std::vector<float> &ary);
} // namespace ns_st10

#endif