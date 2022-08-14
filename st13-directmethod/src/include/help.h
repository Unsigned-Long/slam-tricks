#ifndef HELP_H
#define HELP_H

#define FORMAT_VECTOR
#include "artwork/logger/logger.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <algorithm>
#include <filesystem>

namespace ns_st13 {
  /**
   * \brief a function to get all the filenames in the directory
   * \param directory the directory
   * \return the filenames in the directory
   */
  std::vector<std::string> filesInDir(const std::string &directory);

  void showImg(cv::Mat img, const std::string &imgName = "img");
} // namespace ns_st13

#endif