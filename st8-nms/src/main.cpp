#include "artwork/geometry/point.hpp"
#include "nms.hpp"
#include <random>

float gauss(float x) {
  return std::pow(M_E, -x * x / 16);
}
float gauss2(float x) {
  return std::pow(M_E, -x * x / 100);
}

void test_nms1d() {
  std::vector<float> vals{};
  for (int i = 0; i != 20; ++i) {
    float dis1 = std::abs(7 - i);
    float dis2 = std::abs(16 - i);
    vals.push_back(gauss(dis1) + gauss(dis2));
  }
  LOG_VAR(vals);
  LOG_VAR(ns_st8::nms1d(vals, 3));
  std::default_random_engine e;
  std::uniform_real_distribution<float> u(0.0f, 1.0f);
  for (int i = 0; i != 20; ++i) {
    vals[i] = u(e);
  }
  LOG_VAR(vals);
  LOG_VAR(ns_st8::nms1d(vals, 3));
}

void test_nms2d() {
  {
    auto ps = ns_geo::PointSet2i::randomGenerator(20, 0, 199, 0, 199);
    cv::Mat img(200, 200, CV_8UC1, cv::Scalar(0));
    for (int i = 0; i != img.rows; ++i) {
      for (int j = 0; j != img.cols; ++j) {
        for (const auto &p : ps) {
          float dis = ns_geo::distance(p, ns_geo::Point2i(i, j));
          img.at<uchar>(i, j) += (uchar)(150.0f * gauss2(dis));
        }
      }
    }
    auto max = ns_st8::nms2d(img, 3);
    LOG_VAR(max);

    cv::Mat color;
    cv::cvtColor(img, color, cv::COLOR_GRAY2BGR);
    for (const auto &elem : max) {
      cv::drawMarker(color, elem, cv::Scalar(0, 0, 255), cv::MarkerTypes::MARKER_CROSS, 5, 1);
    }
    cv::namedWindow("win", cv::WindowFlags::WINDOW_FREERATIO);
    cv::imshow("win", color);
    cv::waitKey(0);
    cv::imwrite("../img/2d/norm_nms2d_3.png", color);
    cv::imwrite("../img/2d/norm_nms2d_src.png", img);
  }
  {
    cv::Mat img(20, 20, CV_8UC1);
    std::default_random_engine e;
    std::normal_distribution<float> n(127, 30);
    for (int i = 0; i != img.rows; ++i) {
      for (int j = 0; j != img.cols; ++j) {
        img.at<uchar>(i, j) = n(e);
      }
    }
    auto max = ns_st8::nms2d(img, 3);
    LOG_VAR(max);

    cv::resize(img, img, cv::Size(200, 200), 0.0, 0.0, cv::InterpolationFlags::INTER_NEAREST);
    cv::Mat color;
    cv::cvtColor(img, color, cv::COLOR_GRAY2BGR);
    for (const auto &elem : max) {
      cv::drawMarker(color, elem * 10 + cv::Point2i(5, 5), cv::Scalar(0, 0, 255), cv::MarkerTypes::MARKER_CROSS, 5, 1);
    }
    cv::namedWindow("win", cv::WindowFlags::WINDOW_FREERATIO);
    cv::imshow("win", color);
    cv::waitKey(0);
    cv::imwrite("../img/2d/unif_nms2d_3.png", color);
    cv::imwrite("../img/2d/unif_nms2d_src.png", img);
  }
}

int main(int argc, char const *argv[]) {
  test_nms1d();
  // test_nms2d();
  return 0;
}
