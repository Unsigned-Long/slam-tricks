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
    const ushort REFINE_HWS;
    cv::Mat grayImg;
    cv::Mat likehood;
    cv::Mat gradX;
    cv::Mat gradY;
    std::vector<cv::Point> corners;
    
    std::vector<cv::Point2f> corners_sp;
    std::vector<float> scores;
    std::vector<std::pair<float, float>> corners_modes;

  public:
    Detector(ushort protoHWS = 5, ushort nmsHWS = 4, ushort histHWS = 10, ushort refineHWS = 5);

    void solve(cv::Mat gImg);

  protected:
    void compute_likehood();

    void findCorners();

    void verifyCorners();

    void refineCorners();
  };

} // namespace ns_st10

#endif