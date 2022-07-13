#include "prototype.h"
#include "helper.h"

namespace ns_st10 {
  ProtoType::ProtoType(ushort radius, float angle1, float angle2)
      : A(2 * radius - 1, 2 * radius - 1, CV_32FC1, cv::Scalar(0.0)),
        B(2 * radius - 1, 2 * radius - 1, CV_32FC1, cv::Scalar(0.0)),
        C(2 * radius - 1, 2 * radius - 1, CV_32FC1, cv::Scalar(0.0)),
        D(2 * radius - 1, 2 * radius - 1, CV_32FC1, cv::Scalar(0.0)) {
    ushort size = 2 * radius - 1, r = radius - 1;
    for (int i = 0; i != size; ++i) {
      for (int j = 0; j != size; ++j) {
        if (i == r && j == r) {
          continue;
        }
        float dis = std::sqrt((i - r) * (i - r) + (j - r) * (j - r));
        float gaussVal = ns_st10::gauss(dis, 0.0f, radius / 2.0f);
        float angle = std::atan2(i - r, j - r);
        if (inAngleRange(angle, angle1, angle2)) {
          A.at<float>(i, j) = gaussVal;
        } else if (inAngleRange(angle, angle2, angle1 + M_PI)) {
          C.at<float>(i, j) = gaussVal;
        } else if (inAngleRange(angle, angle1 + M_PI, angle2 + M_PI)) {
          B.at<float>(i, j) = gaussVal;
        } else if (inAngleRange(angle, angle2 + M_PI, angle1)) {
          D.at<float>(i, j) = gaussVal;
        }
      }
    }
  }
} // namespace ns_st10
