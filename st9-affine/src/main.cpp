#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "projective.hpp"

void test_projective() {
  int r = 1000, c = 1000;
  cv::Mat img(c, r, CV_8UC3, cv::Scalar(255, 255, 255));

  // draw
  cv::circle(img, cv::Point(c / 2.0, r / 2.0), 200, cv::Scalar(0, 0, 255), 2);
  cv::drawMarker(img, cv::Point(c / 2.0, r / 2.0), cv::Scalar(0, 0, 255), cv::MarkerTypes::MARKER_TILTED_CROSS, 200, 2);

  // the point pairs
  ns_geo::PointSet2d pc1{{c / 2.0, 0}, {c - 1.0, r / 2.0}, {c - 300.0, r - 100.0}, {0, r / 2.0}};
  ns_geo::PointSet2d pc2{{0.0, 0.0}, {c - 1.0, 0.0}, {c - 1.0, r - 1.0}, {0.0, r - 1.0}};

  for (int i = 0; i != pc1.size(); ++i) {
    const auto &p = pc1[i];
    cv::circle(img, cv::Point(p.x, p.y), 5, cv::Scalar(0, 255, 0), 5);
    if (i == pc1.size() - 1) {
      cv::line(img, cv::Point(p.x, p.y), cv::Point(pc1[0].x, pc1[0].y), cv::Scalar(0, 255, 0), 2);
    } else {
      cv::line(img, cv::Point(p.x, p.y), cv::Point(pc1[i + 1].x, pc1[i + 1].y), cv::Scalar(0, 255, 0), 2);
    }
  }

  // from pc2 to pc1
  auto trans = ns_st9::projectiveBackward(pc1, pc2);

  for (int i = 0; i != 4; ++i) {
    const auto &p1 = pc1[i], p2 = pc2[i];
    LOG_VAR(p1, p2, trans(p2));
  }

  // cv::Mat dst(c, r, CV_8UC3, cv::Scalar(255, 255, 255));

  // for (int i = 0; i != dst.rows; ++i) {
  //   for (int j = 0; j != dst.cols; ++j) {
  //     ns_geo::Point2d p();
  //   }
  // }

  cv::namedWindow("win", cv::WINDOW_FREERATIO);
  cv::imshow("win", img);
  cv::waitKey(0);
}

int main(int argc, char const *argv[]) {
  test_projective();
  return 0;
}
