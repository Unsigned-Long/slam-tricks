#ifndef FEATURE_H
#define FEATURE_H

#include "opencv2/features2d.hpp"

namespace ns_st11 {

  void detect(cv::Mat img, std::vector<cv::KeyPoint> &kps, cv::Mat &descriptors);

  void match(cv::Mat &descriptors1, cv::Mat &descriptors2, std::vector<cv::DMatch> &matches);

} // namespace ns_st11

#endif