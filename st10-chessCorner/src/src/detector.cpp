#include "detector.h"

namespace ns_st10 {
  Detector::Detector(ushort radius, ushort hws)
      : PROTO_RADIUS(radius), NMS_HALFWIN_SIZE(hws) {}

  void Detector::solve(cv::Mat grayImg) {
    compute_likehood(grayImg);
    findCorners();
  }

  void Detector::compute_likehood(cv::Mat grayImg) {
    auto proto1 = ns_st10::ProtoType(PROTO_RADIUS, 0.0, M_PI_2);
    auto proto2 = ns_st10::ProtoType(PROTO_RADIUS, M_PI_4, M_PI_4 * 3);
    cv::Mat s_type1, s_type2;
    {
      cv::Mat fA, fB, fC, fD;
      cv::filter2D(grayImg, fA, CV_32FC1, proto1.A);
      cv::filter2D(grayImg, fB, CV_32FC1, proto1.B);
      cv::filter2D(grayImg, fC, CV_32FC1, proto1.C);
      cv::filter2D(grayImg, fD, CV_32FC1, proto1.D);
      cv::Mat mu = 0.25f * (fA + fB + fC + fD);
      cv::Mat minAB = cv::min(fA, fB);
      cv::Mat minCD = cv::min(fC, fD);
      cv::Mat s1 = cv::min(cv::Mat(mu - minAB), cv::Mat(minCD - mu));
      cv::Mat s2 = cv::min(cv::Mat(minAB - mu), cv::Mat(mu - minCD));
      s_type1 = cv::max(s1, s2);
    }
    {
      cv::Mat fA, fB, fC, fD;
      cv::filter2D(grayImg, fA, CV_32FC1, proto2.A);
      cv::filter2D(grayImg, fB, CV_32FC1, proto2.B);
      cv::filter2D(grayImg, fC, CV_32FC1, proto2.C);
      cv::filter2D(grayImg, fD, CV_32FC1, proto2.D);
      cv::Mat mu = 0.25f * (fA + fB + fC + fD);
      cv::Mat minAB = cv::min(fA, fB);
      cv::Mat minCD = cv::min(fC, fD);
      cv::Mat s1 = cv::min(cv::Mat(mu - minAB), cv::Mat(minCD - mu));
      cv::Mat s2 = cv::min(cv::Mat(minAB - mu), cv::Mat(mu - minCD));
      s_type2 = cv::max(s1, s2);
    }
    likehood = cv::max(s_type1, s_type2);
    likehood = cv::max(0.0f, likehood);
  }

  void Detector::findCorners() {
    double maxVal;
    cv::minMaxIdx(likehood, nullptr, &maxVal);
    auto corners_t = nms2d(likehood, NMS_HALFWIN_SIZE);
    for (const auto &pt : corners_t) {
      if (likehood.at<float>(pt) > 0.5 * maxVal &&
          pt.x >= PROTO_RADIUS &&
          pt.y >= PROTO_RADIUS &&
          pt.x < likehood.cols - PROTO_RADIUS &&
          pt.y < likehood.rows - PROTO_RADIUS) {
        corners.push_back(pt);
      }
    }
    // display
    LOG_VAR(corners.size());
    showImg(ns_st10::drawMarks(cvt_32FC1_8UC1(likehood), corners));
    cv::waitKey(0);
  }
} // namespace ns_st10
