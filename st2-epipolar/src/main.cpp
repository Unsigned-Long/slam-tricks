#include "epipolar.hpp"
#include "opencv2/opencv.hpp"
#include <algorithm>

void detectAndMatch(cv::Mat img1, cv::Mat img2,
                    std::vector<cv::KeyPoint> &kps1,
                    std::vector<cv::KeyPoint> &kps2,
                    std::vector<cv::DMatch> &match) {

  cv::Mat descriptors1, ddescriptors2;

  cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
  cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
  cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

  detector->detect(img1, kps1);
  detector->detect(img2, kps2);

  descriptor->compute(img1, kps1, descriptors1);
  descriptor->compute(img2, kps2, ddescriptors2);

  matcher->match(descriptors1, ddescriptors2, match);
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

  {
    // for OpenCV
    std::vector<cv::DMatch> matches;

    double min_dist = std::min_element(match.cbegin(), match.cend(), [](const cv::DMatch &m1, const cv::DMatch &m2) {
                        return m1.distance < m2.distance;
                      })->distance;

    for (int i = 0; i < match.size(); i++) {
      if (match[i].distance < std::max(2.0 * min_dist, 30.0)) {
        matches.push_back(match[i]);
      }
    }

    // cv::Mat matchImg;
    // cv::drawMatches(img1, kps1, img2, kps2, matches, matchImg);
    // cv::imshow("match", matchImg);
    // cv::waitKey(0);
    // cv::imwrite("../img/select-matches-opencv.png", matchImg);

    std::vector<cv::Point2f> pts1;
    std::vector<cv::Point2f> pts2;
    for (int i = 0; i < (int)matches.size(); i++) {
      pts1.push_back(kps1[matches[i].queryIdx].pt);
      pts2.push_back(kps2[matches[i].trainIdx].pt);
    }

    auto f = cv::findFundamentalMat(pts1, pts2, cv::FM_8POINT);

    std::cout << "---------------" << std::endl;
    std::cout << "Solve By OpenCV" << std::endl;
    std::cout << "---------------" << std::endl;
    std::cout << "matched points: " << matches.size() << std::endl;
    std::cout << f << std::endl;

    // debug
    cv::Mat p1 = (cv::Mat_<double>(3, 1) << kps1.at(matches.front().queryIdx).pt.x, kps1.at(matches.front().queryIdx).pt.y, 1.0);
    cv::Mat p2 = (cv::Mat_<double>(3, 1) << kps2.at(matches.front().trainIdx).pt.x, kps2.at(matches.front().trainIdx).pt.y, 1.0);
    cv::Mat temp = p2.t() * f;
    double a = temp.at<double>(0, 0);
    double b = temp.at<double>(0, 1);
    double c = temp.at<double>(0, 2);
    std::cout << "p1: " << p1.t() << std::endl;
    std::cout << "p2: " << p2.t() << std::endl;
    std::cout << "'a,b,c': " << a << ", " << b << ", " << c << std::endl;
    double num = a * kps1.at(matches.front().queryIdx).pt.x + b * kps1.at(matches.front().queryIdx).pt.y + c;
    std::cout << "'num': " << num << std::endl;
    std::cout << "den: " << a * a + b * b << std::endl;
    std::cout << "'num*num/den': " << num * num / (a * a + b * b) << std::endl;
    std::cout << "------------------------" << std::endl;
    auto f2 = ns_st2::solveEpipolar(kps1, kps2, matches);
    std::cout << f2 << std::endl;
  }
  {
    // for our method
    
  }
  return 0;
}
