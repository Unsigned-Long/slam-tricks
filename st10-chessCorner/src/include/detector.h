#ifndef DETECTOR_H
#define DETECTOR_H

#include "helper.h"
#include "prototype.h"

namespace ns_st10 {
  class Detector {
  private:
    const ushort PROTO_RADIUS;
    const ushort NMS_HALFWIN_SIZE;
    cv::Mat likehood;
    std::vector<cv::Point> corners;

  public:
    Detector(ushort radius, ushort hws);

    void solve(cv::Mat grayImg);

  protected:
    void compute_likehood(cv::Mat grayImg);

    void findCorners();
  };

} // namespace ns_st10

#endif