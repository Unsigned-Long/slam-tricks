#include "artwork/timer/timer.h"
#include "epipolar.hpp"
#include "opencv2/opencv.hpp"
#include <algorithm>

void detectAndMatch(cv::Mat img1, cv::Mat img2,
                    std::vector<cv::KeyPoint> &kps1,
                    std::vector<cv::KeyPoint> &kps2,
                    std::vector<cv::DMatch> &match) {

  cv::Mat descriptors1, descriptors2;

  cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
  cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
  cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

  detector->detect(img1, kps1);
  detector->detect(img2, kps2);

  descriptor->compute(img1, kps1, descriptors1);
  descriptor->compute(img2, kps2, descriptors2);

  matcher->match(descriptors1, descriptors2, match);
  return;
}

int main(int argc, char const *argv[]) {
  float fx = 520.9, cx = 325.1, fy = 521.0, cy = 249.7;
  CameraInnerParam innerParam(fx, fy, cx, cy);

  cv::Mat img1 = cv::imread("../img/1.png", cv::IMREAD_COLOR);
  cv::Mat img2 = cv::imread("../img/2.png", cv::IMREAD_COLOR);

  std::vector<cv::KeyPoint> kps1, kps2;
  std::vector<cv::DMatch> match;

  detectAndMatch(img1, img2, kps1, kps2, match);

  ns_timer::Timer timer;

  {
    // for OpenCV
    std::cout << "---------------" << std::endl;
    std::cout << "Solve By OpenCV" << std::endl;
    std::cout << "---------------" << std::endl;

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

    auto f = cv::findFundamentalMat(pts1, pts2, cv::FM_8POINT);

    std::cout << timer.last_elapsed("cost time") << std::endl;
    std::cout << "matched points: " << goodMatches.size() << std::endl;
    std::cout << "fun matrix:\n"
              << f << std::endl;

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

  {
    // for our method

    std::cout << "---------------" << std::endl;
    std::cout << "Solve By Ourself" << std::endl;
    std::cout << "---------------" << std::endl;

    std::vector<cv::DMatch> goodMatches;
    timer.re_start();
    auto f = ns_st2::solveEpipolar(kps1, kps2, match, &goodMatches);
    std::cout << timer.last_elapsed("cost time") << std::endl;
    std::cout << "matched points: " << goodMatches.size() << std::endl;
    std::cout << "fun matrix:\n"
              << f << std::endl;

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
  }
  return 0;
}
