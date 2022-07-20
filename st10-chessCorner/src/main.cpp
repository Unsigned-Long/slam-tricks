#include "detector.h"

int main(int argc, char const *argv[]) {
  // cv::Mat img = cv::imread("../img/small_2.jpg");
  cv::Mat img = cv::imread("../img/img2.png");
  cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
  auto solver = ns_st10::Detector();
  solver.solve(img, true);
  return 0;
}
