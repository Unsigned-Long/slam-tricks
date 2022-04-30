#include "artwork/timer/timer.h"
#include "epipolar.hpp"
#include "opencv2/opencv.hpp"
#include "recovery.hpp"
#include <algorithm>

const float fx = 520.9, cx = 325.1, fy = 521.0, cy = 249.7;
const cv::Point2d principal_point(325.1, 249.7);
const double focal_length = 521;
// const float fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
// const cv::Point2d principal_point(607.1928, 185.2157);
// const double focal_length = 718.856;
const CameraInnerParam innerParam(fx, fy, cx, cy);

void fMatrixOpenCV(cv::Mat img1, cv::Mat img2,
                   const std::vector<cv::KeyPoint> &kps1,
                   const std::vector<cv::KeyPoint> &kps2,
                   const std::vector<cv::DMatch> &match) {
  // for OpenCV
  std::cout << "---------------" << std::endl;
  std::cout << "Solve By OpenCV" << std::endl;
  std::cout << "---------------" << std::endl;
  ns_timer::Timer timer;

  timer.re_start();

  std::vector<cv::DMatch> goodMatches;

  double min_dist = std::min_element(match.cbegin(), match.cend(), [](const cv::DMatch &m1, const cv::DMatch &m2) {
                      return m1.distance < m2.distance;
                    })->distance;

  for (int i = 0; i < match.size(); i++) {
    if (match[i].distance < std::max(2.0 * min_dist, 30.0)) {
      goodMatches.push_back(match[i]);
    }
  }

  std::vector<cv::Point2f> pts1;
  std::vector<cv::Point2f> pts2;
  for (int i = 0; i < goodMatches.size(); i++) {
    pts1.push_back(kps1[goodMatches[i].queryIdx].pt);
    pts2.push_back(kps2[goodMatches[i].trainIdx].pt);
  }

  auto e = cv::findEssentialMat(pts1, pts2, focal_length, principal_point);
  std::cout << timer.last_elapsed("cost time") << std::endl;

  std::cout << "matched points: " << goodMatches.size() << std::endl;
  std::cout << "essential matrix:\n"
            << e << std::endl;

  std::vector<cv::Point2f> points1;
  std::vector<cv::Point2f> points2;

  for (int i = 0; i < (int)goodMatches.size(); i++) {
    points1.push_back(kps1[goodMatches[i].queryIdx].pt);
    points2.push_back(kps2[goodMatches[i].trainIdx].pt);
  }
  cv::Mat R, t;
  recoverPose(e, points1, points2, R, t, focal_length, principal_point);
  std::cout << "R is " << std::endl
            << R << std::endl;
  std::cout << "t is " << std::endl
            << t << std::endl;

  cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
  auto f = K.inv().t() * e * K.inv();

  float mean = 0.0, sigma = 0.0, errorMeanNorm = 0.0;
  for (int i = 0; i != goodMatches.size(); ++i) {
    const auto &match = goodMatches.at(i);
    cv::Mat p1 = (cv::Mat_<double>(3, 1) << kps1.at(match.queryIdx).pt.x, kps1.at(match.queryIdx).pt.y, 1.0);
    cv::Mat p2 = (cv::Mat_<double>(3, 1) << kps2.at(match.trainIdx).pt.x, kps2.at(match.trainIdx).pt.y, 1.0);
    cv::Mat temp = p2.t() * f * p1;
    mean += temp.at<double>(0, 0);
    errorMeanNorm += std::abs(temp.at<double>(0, 0));
  }
  mean /= goodMatches.size();
  errorMeanNorm /= goodMatches.size();
  std::cout << "mean: " << mean << std::endl;
  std::cout << "errorMeanNorm: " << errorMeanNorm << std::endl;

  for (int i = 0; i != goodMatches.size(); ++i) {
    const auto &match = goodMatches.at(i);
    cv::Mat p1 = (cv::Mat_<double>(3, 1) << kps1.at(match.queryIdx).pt.x, kps1.at(match.queryIdx).pt.y, 1.0);
    cv::Mat p2 = (cv::Mat_<double>(3, 1) << kps2.at(match.trainIdx).pt.x, kps2.at(match.trainIdx).pt.y, 1.0);
    cv::Mat temp = p2.t() * f * p1;
    sigma += std::pow(mean - temp.at<double>(0, 0), 2.0);
  }
  sigma /= goodMatches.size() - 1;
  std::cout << "sigma: " << std::sqrt(sigma) << std::endl;

  cv::Mat matchImg;
  cv::drawMatches(img1, kps1, img2, kps2, goodMatches, matchImg);
  cv::imshow("match", matchImg);
  cv::waitKey(0);
  cv::imwrite("../img/select-matches-opencv.png", matchImg);
}

void fMatrixStatistic(cv::Mat img1, cv::Mat img2,
                      const std::vector<cv::KeyPoint> &kps1,
                      const std::vector<cv::KeyPoint> &kps2,
                      const std::vector<cv::DMatch> &match) {
  // for our method
  ns_timer::Timer timer;

  std::cout << "---------------" << std::endl;
  std::cout << "Solve By Ourself" << std::endl;
  std::cout << "---------------" << std::endl;

  std::vector<cv::DMatch> goodMatches;

  timer.re_start();
  auto e = ns_st2::solveEpipolar(kps1, kps2, match, innerParam, &goodMatches);
  std::cout << timer.last_elapsed("cost time") << std::endl;

  std::cout << "matched points: "
            << goodMatches.size() << std::endl;
  std::cout << "essential matrix:\n"
            << e << std::endl;

  auto K = innerParam.toEigenMatrix();
  auto f = K.inverse().transpose() * e * K.inverse();

  float mean = 0.0, sigma = 0.0, errorMeanNorm = 0.0;
  for (int i = 0; i != goodMatches.size(); ++i) {
    const auto &match = goodMatches.at(i);
    float u1 = kps1.at(match.queryIdx).pt.x, v1 = kps1.at(match.queryIdx).pt.y;
    float u2 = kps2.at(match.trainIdx).pt.x, v2 = kps2.at(match.trainIdx).pt.y;
    Eigen::Vector3f p1(u1, v1, 1.0f);
    Eigen::Vector3f p2(u2, v2, 1.0f);
    mean += (p2.transpose() * f * p1)(0, 0);
    errorMeanNorm += std::abs((p2.transpose() * f * p1)(0, 0));
  }
  mean /= goodMatches.size();
  errorMeanNorm /= goodMatches.size();
  std::cout << "mean: " << mean << std::endl;
  std::cout << "errorMeanNorm: " << errorMeanNorm << std::endl;

  for (int i = 0; i != goodMatches.size(); ++i) {
    const auto &match = goodMatches.at(i);
    float u1 = kps1.at(match.queryIdx).pt.x, v1 = kps1.at(match.queryIdx).pt.y;
    float u2 = kps2.at(match.trainIdx).pt.x, v2 = kps2.at(match.trainIdx).pt.y;
    Eigen::Vector3f p1(u1, v1, 1.0f);
    Eigen::Vector3f p2(u2, v2, 1.0f);
    sigma += std::pow(mean - (p2.transpose() * f * p1)(0, 0), 2.0);
  }
  sigma /= goodMatches.size() - 1;
  std::cout << "sigma: " << std::sqrt(sigma) << std::endl;

  cv::Mat matchImg;
  cv::drawMatches(img1, kps1, img2, kps2, goodMatches, matchImg);
  cv::imshow("match", matchImg);
  cv::waitKey(0);
  cv::imwrite("../img/select-matches-ourself.png", matchImg);

  Eigen::Matrix3f rot21;
  Eigen::Vector3f t21;
  {
    bool b = ns_st2::recoveryMove(e, innerParam.toEigenMatrix(),
                                  kps1.at(goodMatches.front().queryIdx),
                                  kps2.at(goodMatches.front().trainIdx), rot21, t21);
    std::cout << b << std::endl;
    if (b) {
      std::cout << "rot\n";
      std::cout << rot21 << std::endl;
      std::cout << "t\n";
      std::cout << t21.transpose() << std::endl;
      auto depth = ns_st2::triangulation(kps1.at(goodMatches.front().queryIdx),
                                         kps2.at(goodMatches.front().trainIdx), innerParam.toEigenMatrix(),
                                         rot21, t21);
      std::cout << depth.first << ", " << depth.second << std::endl;
    }
  }
}

void recoveryMovement(cv::Mat img1, cv::Mat img2,
                      const std::vector<cv::KeyPoint> &kps1,
                      const std::vector<cv::KeyPoint> &kps2,
                      const std::vector<cv::DMatch> &match) {
  std::vector<cv::DMatch> goodMatches;
  auto e = ns_st2::solveEpipolar(kps1, kps2, match, innerParam, &goodMatches);

  std::cout << e << std::endl;

  Eigen::Matrix3f rot21;
  Eigen::Vector3f t21;
  {
    bool b = ns_st2::recoveryMove(e, innerParam.toEigenMatrix(),
                                  kps1.at(goodMatches.front().queryIdx),
                                  kps2.at(goodMatches.front().trainIdx), rot21, t21);
    std::cout << b << std::endl;
    if (b) {
      std::cout << "rot\n";
      std::cout << rot21 << std::endl;
      std::cout << "t\n";
      std::cout << t21.transpose() << std::endl;
      auto depth = ns_st2::triangulation(kps1.at(goodMatches.front().queryIdx),
                                         kps2.at(goodMatches.front().trainIdx), innerParam.toEigenMatrix(),
                                         rot21, t21);
      std::cout << depth.first << ", " << depth.second << std::endl;
    }
  }
  return;
}

int main(int argc, char const *argv[]) {

  cv::Mat img1 = cv::imread("../img/1.png", cv::IMREAD_COLOR);
  cv::Mat img2 = cv::imread("../img/2.png", cv::IMREAD_COLOR);

  std::vector<cv::KeyPoint> kps1, kps2;
  std::vector<cv::DMatch> match;

  detectAndMatch(img1, img2, kps1, kps2, match);
  // ::fMatrixOpenCV(img1, img2, kps1, kps2, match);
  // ::fMatrixStatistic(img1, img2, kps1, kps2, match);
  ::recoveryMovement(img1, img2, kps1, kps2, match);

  return 0;
}
