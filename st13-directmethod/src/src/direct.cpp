#include "direct.h"

namespace ns_st13 {
  void DirectMethod::solve(const cv::Mat newImg) {
    // assign and update images
    if (!initialized) {
      this->img2 = newImg;
      initialized = true;
      return;
    } else {
      this->img1 = this->img2;
      this->img2 = newImg;
    }
  }

  void DirectMethod::increasePixPts(const cv::Mat &img, std::vector<cv::Point2d> &pixPts) {
  }
} // namespace ns_st13
