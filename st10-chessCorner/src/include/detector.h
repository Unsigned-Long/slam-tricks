#ifndef DETECTOR_H
#define DETECTOR_H

#include "helper.h"
#include "prototype.h"

namespace ns_st10 {
  class Detector {
  private:
    const ushort PROTO_HWS;
    const ushort NMS_HWS;
    const ushort HIST_HWS;
    cv::Mat grayImg;
    cv::Mat likehood;
    std::vector<cv::Point> corners;

  public:
    Detector(ushort protoHWS = 5, ushort nmsHWS = 4, ushort histHWS = 10);

    void solve(cv::Mat gImg);

  protected:
    void compute_likehood();

    void findCorners();

    void verifyCorners();
  };

} // namespace ns_st10

#endif