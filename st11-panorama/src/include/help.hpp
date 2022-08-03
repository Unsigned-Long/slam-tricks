#ifndef HELP_HPP
#define HELP_HPP

#include "artwork/logger/logger.h"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <filesystem>
#include <random>

namespace ns_st11 {
  static void showImg(cv::Mat img, const std::string &name = "img") {
    cv::namedWindow(name, cv::WINDOW_FREERATIO);
    cv::imshow(name, img);
    cv::waitKey(0);
  }

  static cv::Mat drawMarker(cv::Mat img, const std::vector<cv::KeyPoint> &kps) {
    cv::Mat map = img.clone();
    cv::drawKeypoints(img, kps, map);
    return map;
  }

  static std::vector<std::string> filesInDir(const std::string &directory) {
    std::vector<std::string> files;
    for (const auto &elem : std::filesystem::directory_iterator(directory))
      if (elem.status().type() != std::filesystem::file_type::directory)
        files.push_back(std::filesystem::canonical(elem.path()).c_str());
    std::sort(files.begin(), files.end());
    return files;
  }

  static cv::Mat drawMatches(cv::Mat img1, cv::Mat img2,
                             const std::vector<cv::KeyPoint> &kps1, const std::vector<cv::KeyPoint> &kps2,
                             const std::vector<cv::DMatch> &matches) {
    cv::Mat map;
    cv::drawMatches(img1, kps1, img2, kps2, matches, map);
    return map;
  }

  template <typename ElemType, typename Comp>
  std::vector<std::size_t> kBest(const std::vector<ElemType> &vec, std::size_t k, Comp comp) {
    auto insertPos = [&vec, &comp](const std::vector<std::size_t> &idxAry, const ElemType &tarVal) {
      int low = 0, high = idxAry.size() - 1, p;
      while (true) {
        int mid = (low + high) / 2;
        const ElemType &midVal = vec[idxAry[mid]];
        if (comp(tarVal, midVal)) {
          high = mid - 1;
        } else if (comp(midVal, tarVal)) {
          low = mid + 1;
        } else {
          p = mid;
          break;
        }
        if (low > high) {
          p = low;
          break;
        }
      }
      return p;
    };

    // init the best k vector
    std::vector<std::size_t> bestVec(k);
    for (int i = 0; i != k; ++i) {
      bestVec[i] = i;
    }
    std::sort(bestVec.begin(), bestVec.end(), [&vec, &comp](auto idx1, auto idx2) {
      return comp(vec[idx1], vec[idx2]);
    });
    // for each
    for (int i = bestVec.size(); i != vec.size(); ++i) {
      const auto &curElem = vec[i];
      // find cur element pos in the best vector
      std::size_t p = insertPos(bestVec, curElem);
      if (p == bestVec.size()) {
        continue;
      }
      // insert
      for (int j = bestVec.size() - 1; j != p; --j) {
        bestVec[j] = bestVec[j - 1];
      }
      bestVec[p] = i;
    }
    return bestVec;
  }

  static std::vector<std::size_t> samplingWoutReplace(std::default_random_engine &engine,
                                                      std::size_t num,
                                                      std::size_t start,
                                                      std::size_t end,
                                                      std::size_t step = 1) {
    // create the pool for sampling
    std::vector<std::size_t> idxPool((end - start) / step + 1);
    for (int i = 0; i != idxPool.size(); ++i) {
      idxPool.at(i) = start + i * step;
    }
    std::vector<std::size_t> res(num);
    // the engine
    for (std::size_t i = 0; i != num; ++i) {
      // generate the random index
      std::uniform_int_distribution<std::size_t> ui(0, idxPool.size() - 1);
      std::size_t ridx = ui(engine);
      // record it
      res.at(i) = idxPool.at(ridx);
      // remove it
      idxPool.at(ridx) = idxPool.back();
      idxPool.pop_back();
    }
    return res;
  }

  template <typename ElemType>
  std::vector<std::size_t> samplingWoutReplace(std::default_random_engine &engine,
                                               const std::vector<ElemType> &dataVec,
                                               std::size_t num) {
    return samplingWoutReplace(engine, num, 0, dataVec.size() - 1, 1);
  }

  template <typename ElemType>
  std::vector<ElemType> samplingWoutReplace2(std::default_random_engine &engine,
                                             const std::vector<ElemType> &dataVec,
                                             std::size_t num) {
    std::vector<std::size_t> res = samplingWoutReplace(engine, dataVec, num);
    std::vector<ElemType> samples(num);
    for (int i = 0; i != num; ++i) {
      samples.at(i) = dataVec.at(res.at(i));
    }
    return samples;
  }

  static cv::Mat drawPairs(cv::Mat img1, cv::Mat img2,
                           const std::vector<cv::Point2d> &pc1,
                           const std::vector<cv::Point2d> &pc2) {
    CV_Assert(pc1.size() == pc2.size());
    std::size_t size = pc1.size();

    std::vector<cv::DMatch> tempMatches(size);
    std::vector<cv::KeyPoint> tempPC1(size), tempPC2(size);

    for (int j = 0; j != size; ++j) {
      tempPC1[j].pt = pc1[j];
      tempPC2[j].pt = pc2[j];
      tempMatches[j].queryIdx = j;
      tempMatches[j].trainIdx = j;
    }

    return drawMatches(img1, img2, tempPC1, tempPC2, tempMatches);
  }
} // namespace ns_st11

#endif