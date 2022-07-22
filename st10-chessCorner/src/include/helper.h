#ifndef HELPER_H
#define HELPER_H

#define FORMAT_VECTOR
#include "artwork/logger/logger.h"
#include "artwork/timer/timer.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <deque>

namespace ns_st10 {
  // convert a float image to a gray image
  cv::Mat cvt_32FC1_8UC1(cv::Mat img_f);

  // gaussian function
  float gauss(float x, float mean, float sigma);

  // display an image
  void showImg(cv::Mat img, const std::string &name = "img");

  // judge an angle is in range [startAng, endAng]
  bool inAngleRange(float angle, float startAng, float endAng);

  // Non maximum suppression algorithm for two-dimensional image
  std::vector<cv::Point2i> nms2d(const cv::Mat img, const ushort hws);

  // draw markers in a gray image
  cv::Mat drawMarks(cv::Mat grayImg, const std::vector<cv::Point> &pts);

  // draw markers in a gray image
  cv::Mat drawMarks(cv::Mat grayImg, const std::vector<cv::Point2f> &pts);

  // draw modes [vectors] in a gray image
  cv::Mat drawModes(cv::Mat grayImg, const std::vector<cv::Point> &pts,
                    const std::vector<std::pair<float, float>> &modes);

  // draw modes [vectors] in a gray image
  cv::Mat drawModes(cv::Mat grayImg, const std::vector<cv::Point2f> &pts,
                    const std::vector<std::pair<float, float>> &modes);

  // Gaussian smoothing algorithm for one-dimensional data
  void gaussFilter(std::vector<float> &ary);

  // mean shift algorithm for one-dimensional data
  std::pair<std::size_t, std::size_t> meanShift(const std::vector<float> &ary);

  // daw a chess board in an image
  cv::Mat drawChessBoard(cv::Mat grayImg,
                         const std::deque<std::deque<std::size_t>> &board,
                         const std::vector<cv::Point2f> &corner);

  // daw a chess board in an image
  cv::Mat drawChessBoard(cv::Mat grayImg,
                         const std::vector<std::vector<cv::Point2f>> &board);
} // namespace ns_st10

#endif