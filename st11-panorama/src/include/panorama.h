#ifndef PANORAMA_H
#define PANORAMA_H

#include "artwork/logger/logger.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

namespace ns_st11 {

  cv::Mat panorama(const std::vector<cv::Mat> &grayImgs);

  cv::Mat panorama(const std::vector<std::string> &grayImgNames);

} // namespace ns_st11

#endif