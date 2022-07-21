#include "detector.h"

int main(int argc, char const *argv[]) {
  // cv::Mat img = cv::imread("../img/img.png");
  // cv::Mat img = cv::imread("../img/small_1.jpg");
  // cv::Mat img = cv::imread("../img/small_2.jpg");
  // cv::Mat img = cv::imread("../img/img2.png");
  cv::Mat img = cv::imread("../img/img2_r.png");
  cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
  auto solver = ns_st10::Detector();
  auto res = solver.solve(img, true);
  if (res.first) {
    res.second.write("../output/corners.txt");
  }
  return 0;
}
