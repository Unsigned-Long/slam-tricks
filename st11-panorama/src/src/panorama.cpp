#include "panorama.h"
#include "feature.h"
#include "help.hpp"
#include "projective.h"

namespace ns_st11 {
  cv::Mat panorama(const std::vector<std::string> &grayImgNames) {
    CV_Assert(grayImgNames.size() >= 2);
    std::size_t size = grayImgNames.size();
    std::vector<cv::Mat> imgs(size);
    for (int i = 0; i != size; ++i) {
      imgs[i] = cv::imread(grayImgNames[i], cv::IMREAD_GRAYSCALE);
    }
    return panorama(imgs);
  }

  cv::Mat panorama(const std::vector<cv::Mat> &grayImgs) {
    CV_Assert(grayImgs.size() >= 2);
    std::size_t size = grayImgs.size();

    std::vector<std::vector<cv::KeyPoint>> keyPtsVec(size);
    std::vector<cv::Mat> descriptors(size);
    std::vector<std::vector<cv::DMatch>> matchesVec(size - 1);
    std::vector<std::vector<Eigen::Matrix3d>> homoMats(size, std::vector<Eigen::Matrix3d>(size));

    LOG_PROCESS("detecting key points");
    for (int i = 0; i != size; ++i) {
      const auto &img = grayImgs[i];
      LOG_INFO("detecting key points in image [" + std::to_string(i) + ']');
      ns_st11::detect(img, keyPtsVec[i], descriptors[i]);
      LOG_PLAINTEXT("key points num: " + std::to_string(keyPtsVec[i].size()));
    }

    LOG_PROCESS("matching key points");
    for (int i = 0; i != size - 1; ++i) {
      LOG_INFO("matching key points between imags [" + std::to_string(i) +
               "] and image [" + std::to_string(i + 1) + ']');

      std::vector<cv::DMatch> matches;

      ns_st11::match(descriptors[i], descriptors[i + 1], matches);

      auto idxVec = ns_st11::kBest(matches, std::min(100UL, matches.size()),
                                   [](const cv::DMatch &m1, const cv::DMatch &m2) {
                                     return m1.distance < m2.distance;
                                   });

      matchesVec[i].resize(idxVec.size());
      for (int j = 0; j != idxVec.size(); ++j) {
        matchesVec[i][j] = matches[idxVec[j]];
      }
      LOG_PLAINTEXT("good matches num: " + std::to_string(matchesVec[i].size()));

      // const auto &img1 = grayImgs[i];
      // const auto &img2 = grayImgs[i + 1];
      // showImg(drawMatches(img1, img2, keyPtsVec[i], keyPtsVec[i + 1], matchesVec[i]));
    }

    LOG_PROCESS("computing homo matrix");
    for (int i = 0; i != size - 1; ++i) {
      LOG_INFO("computing homo matrix from imags [" + std::to_string(i + 1) +
               "] and image [" + std::to_string(i) + ']');

      const auto &matches = matchesVec[i];
      const auto &kps1 = keyPtsVec[i], &kps2 = keyPtsVec[i + 1];

      std::vector<cv::Point2d> pc1(matches.size()), pc2(matches.size());
      for (int j = 0; j != matches.size(); ++j) {
        const auto &match = matches[j];
        pc1[j] = kps1[match.queryIdx].pt;
        pc2[j] = kps2[match.trainIdx].pt;
      }

      const float errorThd = 4;
      homoMats[i][i + 1] = solveHomoMatByRANSAC(pc1, pc2, errorThd);

      {
        // display
        // const auto &img1 = grayImgs[i];
        // const auto &img2 = grayImgs[i + 1];
        // showImg(drawPairs(img1, img2, pc1, pc2));

        // pc1.clear(), pc2.clear();
        // for (int j = 0; j != matches.size(); ++j) {
        //   const auto &match = matches[j];
        //   const auto &p1 = kps1[match.queryIdx].pt;
        //   const auto &p2 = kps2[match.trainIdx].pt;

        //   auto p2_pred = projectHomoMat(p1, homoMats[i][i + 1]);

        //   float errorSquard = (p2.x - p2_pred.x) * (p2.x - p2_pred.x) +
        //                       (p2.y - p2_pred.y) * (p2.y - p2_pred.y);
        //   if (errorSquard < errorThd * errorThd) {
        //     pc1.push_back(p1), pc2.push_back(p2);
        //   }
        // }
        // showImg(drawPairs(img1, img2, pc1, pc2));
      }
    }

    LOG_PROCESS("organizing homo matrix");
    for (int i = 0; i != size - 1; ++i) {
      homoMats[i + 1][i] = homoMats[i][i + 1].inverse();
    }
    for (int i = 0; i != size; ++i) {
      homoMats[i][i] = Eigen::Matrix3d::Identity();
    }
    for (int i = 0; i != size - 2; ++i) {
      for (int j = i + 2; j != size; ++j) {
        homoMats[i][j] = homoMats[i][j - 1] * homoMats[j - 1][j];
        homoMats[j][i] = homoMats[i][j].inverse();
      }
    }

    return cv::Mat();
  }
} // namespace ns_st11