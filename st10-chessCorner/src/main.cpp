#include "detector.h"

int main(int argc, char const *argv[]) {
  // cv::Mat img = cv::imread("../img/img.png");
  // cv::Mat img = cv::imread("../img/small_1.jpg");
  cv::Mat img = cv::imread("../img/small_2.jpg");
  // cv::Mat img = cv::imread("../img/img2.png");
  // cv::Mat img = cv::imread("../img/img2_r.png");
  // cv::Mat img = cv::imread("../img/cb1.jpeg");
  // cv::Mat img = cv::imread("../img/cb2.jpeg");
  // cv::Mat img = cv::imread("../img/cb3.jpeg");
  // cv::Mat img = cv::imread("../img/cb4.jpeg");
  cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
  auto solver = ns_st10::Detector();
  auto res = solver.solve(img, true);
  if (res.first) {
    res.second.write("../output/corners.txt");
  }
  auto cb = ns_st10::CBCorners::read("../output/corners.txt");
  // solver.solveMutiCB(img);
  return 0;
}
