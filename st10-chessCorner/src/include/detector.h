#ifndef DETECTOR_H
#define DETECTOR_H

#include "helper.h"
#include "prototype.h"

namespace ns_st10 {
  class Detector {
  private:
    const ushort PROTO_RADIUS;
    cv::Mat likehood;

  public:
    Detector(ushort radius);

    void solve(cv::Mat grayImg);

  protected:
    void compute_likehood(cv::Mat grayImg);
  };

} // namespace ns_st10

#endif