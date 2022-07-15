#include "detector.h"

namespace ns_st10 {
  Detector::Detector(ushort protoHWS, ushort nmsHWS, ushort histHWS)
      : PROTO_HWS(protoHWS), NMS_HWS(nmsHWS), HIST_HWS(histHWS) {}

  void Detector::solve(cv::Mat gImg) {
    this->grayImg = gImg;
    compute_likehood();
    findCorners();
    showImg(ns_st10::drawMarks(cvt_32FC1_8UC1(likehood), corners), "old corners");
    cv::waitKey(0);
    verifyCorners();
    showImg(ns_st10::drawMarks(cvt_32FC1_8UC1(likehood), corners), "new corners");
    cv::waitKey(0);
  }

  void Detector::compute_likehood() {
    auto proto1 = ns_st10::ProtoType(PROTO_HWS, 0.0, M_PI_2);
    auto proto2 = ns_st10::ProtoType(PROTO_HWS, M_PI_4, M_PI_4 * 3);
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
    cv::normalize(likehood, likehood);
  }

  void Detector::findCorners() {
    double maxVal;
    cv::minMaxIdx(likehood, nullptr, &maxVal);
    auto corners_t = nms2d(likehood, NMS_HWS);
    for (const auto &pt : corners_t) {
      if (likehood.at<float>(pt) > 0.5 * maxVal &&
          pt.x >= HIST_HWS &&
          pt.y >= HIST_HWS &&
          pt.x < likehood.cols - HIST_HWS &&
          pt.y < likehood.rows - HIST_HWS) {
        corners.push_back(pt);
      }
    }
    // display
    LOG_VAR(corners.size());
  }

  void Detector::verifyCorners() {
    ushort rows = grayImg.rows, cols = grayImg.cols;
    cv::Mat gradX, gradY;
    cv::Sobel(grayImg, gradX, CV_32FC1, 1, 0);
    cv::Sobel(grayImg, gradY, CV_32FC1, 0, 1);
    cv::Mat grad, angle;
    cv::cartToPolar(gradX, gradY, grad, angle);
    const float binSize = M_PI / 32;
    std::vector<cv::Point> corners_new;
    std::vector<float> scores_new;
    for (const auto &pt : corners) {
      int x = pt.x, y = pt.y;
      std::vector<float> bins(32, 0.0f);
      for (int i = y - HIST_HWS; i != y + HIST_HWS + 1; ++i) {
        auto gradPtr = grad.ptr<float>(i);
        auto anglePtr = angle.ptr<float>(i);
        for (int j = x - HIST_HWS; j != x + HIST_HWS + 1; ++j) {
          float g = gradPtr[j], a = anglePtr[j];
          if (a >= M_PI) {
            a -= M_PI;
          }
          bins[int(a / binSize)] += g;
        }
      }
      // gauss filter
      gaussFilter(bins);
      // mean shift
      auto modes = meanShift(bins);
      // LOG_VAR(modes.first, modes.second);
      int m1 = modes.first, m2 = modes.second;
      float v1 = bins[m1], v2 = bins[m2];
      int dis = std::abs(m1 - m2);
      if (dis > 16) {
        dis = 32 - dis;
      }
      if (dis > 10 && std::abs(v1 - v2) < std::max(v1, v2) * 0.5f) {
        // compute score
        auto proto = ProtoType(HIST_HWS, std::min(m1, m2) / 31.0f * M_PI, std::max(m1, m2) / 31.0f * M_PI);
        {
          cv::Mat gx, gy;
          cv::Sobel(proto.A, gx, CV_32FC1, 1, 0);
          cv::Sobel(proto.A, gy, CV_32FC1, 0, 1);
          cv::cartToPolar(gx, gy, proto.A, gx);

          cv::Sobel(proto.B, gx, CV_32FC1, 1, 0);
          cv::Sobel(proto.B, gy, CV_32FC1, 0, 1);
          cv::cartToPolar(gx, gy, proto.B, gx);

          cv::Sobel(proto.C, gx, CV_32FC1, 1, 0);
          cv::Sobel(proto.C, gy, CV_32FC1, 0, 1);
          cv::cartToPolar(gx, gy, proto.C, gx);

          cv::Sobel(proto.D, gx, CV_32FC1, 1, 0);
          cv::Sobel(proto.D, gy, CV_32FC1, 0, 1);
          cv::cartToPolar(gx, gy, proto.D, gx);
        }
        cv::Mat subGrad = grad(cv::Range(y - HIST_HWS, y + HIST_HWS + 1), cv::Range(x - HIST_HWS, x + HIST_HWS + 1));
        cv::Mat subExpGrad = cv::max(cv::Mat(cv::max(proto.A, proto.B)), cv::Mat(cv::max(proto.C, proto.D)));
        float score_grad = subGrad.dot(subExpGrad) / (cv::norm(subGrad) * cv::norm(subExpGrad));
        float score_likehood = likehood.at<float>(pt);
        // is a good corner
        corners_new.push_back(pt);
        scores_new.push_back(score_likehood * score_grad);
      }
      {
        // draw
        // std::fstream file = std::fstream("../pyDrawer/hist.csv", std::ios::out);
        // for (int i = 0; i != bins.size() - 1; ++i) {
        //   file << bins[i] << ',';
        // }
        // file << bins[bins.size() - 1];
        // file.close();
        // showImg(ns_st10::drawMarks(cvt_32FC1_8UC1(likehood), {pt}));
        // cv::waitKey(0);
        // int re = system("/bin/python3 /home/csl/CppWorks/artwork/slam-tricks/st10-chessCorner/pyDrawer/drawer.py");
      }
    }
    float score_max = *std::max_element(scores_new.cbegin(), scores_new.cend());
    corners.clear();
    scores.clear();
    for (int i = 0; i != corners_new.size(); ++i) {
      if (scores_new[i] > 0.5f * score_max) {
        corners.push_back(corners_new[i]);
        scores.push_back(scores_new[i]);
      }
    }
  }
} // namespace ns_st10
