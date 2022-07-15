#ifndef PROTOTYPE_H
#define PROTOTYPE_H

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

namespace ns_st10 {
  struct ProtoType {
  public:
    cv::Mat A, B, C, D;

  public:
    ProtoType(ushort radius, float angle1, float angle2);

  };
} // namespace ns_st10

#endif