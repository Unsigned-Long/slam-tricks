#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "projective.hpp"

void test_projective() {
  int r = 1000, c = 1000, thickness = 10;
  cv::Mat src(c, r, CV_8UC3, cv::Scalar(255, 252, 198));

  // draw
  cv::circle(src, cv::Point(c / 2.0, r / 2.0), 200, cv::Scalar(0, 0, 255), thickness);
  cv::drawMarker(src, cv::Point(c / 2.0, r / 2.0), cv::Scalar(0, 0, 255), cv::MarkerTypes::MARKER_TILTED_CROSS, 200, thickness);

  // the point pairs
  ns_geo::PointSet2d pc1{{c / 2.0, 0}, {c - 1.0, r / 2.0}, {c - 300.0, r - 100.0}, {0, r / 2.0}};
  ns_geo::PointSet2d pc2{{0.0, 0.0}, {c - 1.0, 0.0}, {c - 1.0, r - 1.0}, {0.0, r - 1.0}};

  for (int i = 0; i != pc1.size(); ++i) {
    const auto &p = pc1[i];
    if (i == pc1.size() - 1) {
      cv::line(src, cv::Point(p.x, p.y), cv::Point(pc1[0].x, pc1[0].y), cv::Scalar(0, 255, 0), thickness);
      cv::circle(src, cv::Point(pc1[0].x, pc1[0].y), 5, cv::Scalar(255, 0, 0), thickness);
    } else {
      cv::line(src, cv::Point(p.x, p.y), cv::Point(pc1[i + 1].x, pc1[i + 1].y), cv::Scalar(0, 255, 0), thickness);
    }
    cv::circle(src, cv::Point(p.x, p.y), 5, cv::Scalar(255, 0, 0), thickness);
  }

  // from pc2 to pc1
  auto trans = ns_st9::projectiveBackward(pc1, pc2);

  cv::Mat dst(c, r, CV_8UC3, cv::Scalar(255, 255, 255));

  for (int i = 0; i != dst.rows; ++i) {
    for (int j = 0; j != dst.cols; ++j) {
      ns_geo::Point2d p(j, i);
      auto tp = trans(p);
      int ti = tp.y + 0.5, tj = tp.x + 0.5;
      dst.at<cv::Vec3b>(i, j) = src.at<cv::Vec3b>(ti, tj);
    }
  }

  cv::namedWindow("win1", cv::WINDOW_FREERATIO);
  cv::imshow("win1", src);
  cv::namedWindow("win2", cv::WINDOW_FREERATIO);
  cv::imshow("win2", dst);
  cv::imwrite("../img/src.png", src);
  cv::imwrite("../img/dst.png", dst);
  cv::waitKey(0);
}

void test_projective_pic() {
  int r = 500, c = 400, thickness = 4;

  cv::Mat src = cv::imread("../img/src2.png");
  cv::Mat dst(r, c, CV_8UC3, cv::Scalar(255, 255, 255));

  // the point pairs
  ns_geo::PointSet2d pc1{{455, 84}, {770, 130}, {686, 544}, {314, 467}};
  ns_geo::PointSet2d pc2{{0.0, 0.0}, {c - 1.0, 0.0}, {c - 1.0, r - 1.0}, {0.0, r - 1.0}};

  for (int i = 0; i != pc1.size(); ++i) {
    const auto &p = pc1[i];
    if (i == pc1.size() - 1) {
      cv::line(src, cv::Point(p.x, p.y), cv::Point(pc1[0].x, pc1[0].y), cv::Scalar(0, 0, 255), thickness);
      cv::circle(src, cv::Point(pc1[0].x, pc1[0].y), 5, cv::Scalar(255, 0, 0), thickness);
    } else {
      cv::line(src, cv::Point(p.x, p.y), cv::Point(pc1[i + 1].x, pc1[i + 1].y), cv::Scalar(0, 0, 255), thickness);
    }
    cv::circle(src, cv::Point(p.x, p.y), 5, cv::Scalar(255, 0, 0), thickness);
  }

  // from pc2 to pc1
  auto trans = ns_st9::projectiveBackward(pc1, pc2);

  for (int i = 0; i != dst.rows; ++i) {
    for (int j = 0; j != dst.cols; ++j) {
      ns_geo::Point2d p(j, i);
      auto tp = trans(p);
      int ti = tp.y + 0.5, tj = tp.x + 0.5;
      dst.at<cv::Vec3b>(i, j) = src.at<cv::Vec3b>(ti, tj);
    }
  }

  cv::imshow("win1", src);
  cv::imshow("win2", dst);
  cv::imwrite("../img/src2.png", src);
  cv::imwrite("../img/dst2.png", dst);
  cv::waitKey(0);
}

int main(int argc, char const *argv[]) {
  // test_projective();
  test_projective_pic();
  return 0;
}
