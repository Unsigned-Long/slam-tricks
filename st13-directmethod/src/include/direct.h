#ifndef DIRECT_H
#define DIRECT_H

#include "help.h"

namespace ns_st13 {
  class DirectMethod {
    cv::Mat img1;
    cv::Mat img2;
    bool initialized;
    std::vector<cv::Point2d> pixPts1;

  public:
    DirectMethod() : initialized(false){};

    void solve(const cv::Mat newImg);

  protected:
    void increasePixPts(const cv::Mat &img, std::vector<cv::Point2d> &pixPts);
  };
} // namespace ns_st13

#endif