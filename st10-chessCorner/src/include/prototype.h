#ifndef PROTOTYPE_H
#define PROTOTYPE_H

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

namespace ns_st10 {
  struct ProtoType {
  public:
    // Four different prototypes
    cv::Mat A, B, C, D;

  public:
    /**
     * @brief Construct a new ProtoType object
     * 
     * @param radius the radius
     * @param angle1 the start angle [radian]
     * @param angle2 the end angle [radian]
     */
    ProtoType(ushort radius, float angle1, float angle2);
  };
} // namespace ns_st10

#endif