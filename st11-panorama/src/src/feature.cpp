#include "feature.h"

namespace ns_st11 {
  void detect(cv::Mat img, std::vector<cv::KeyPoint> &kps, cv::Mat &descriptors) {

    cv::Ptr<cv::FeatureDetector> detector = cv::SIFT::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::SIFT::create();

    // detector
    detector->detect(img, kps);

    // compute
    descriptor->compute(img, kps, descriptors);
  }

  void match(cv::Mat &descriptors1, cv::Mat &descriptors2, std::vector<cv::DMatch> &matches) {
    cv::Ptr<cv::DescriptorMatcher> matcher =
        cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_L1);

    // match
    matcher->match(descriptors1, descriptors2, matches);
  }

} // namespace ns_st11