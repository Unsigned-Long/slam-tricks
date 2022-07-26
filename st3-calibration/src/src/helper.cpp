#include "helper.h"

namespace ns_st3 {
  std::vector<std::string> filesInDir(const std::string &directory) {
    std::vector<std::string> files;
    for (const auto &elem : std::filesystem::directory_iterator(directory))
      if (elem.status().type() != std::filesystem::file_type::directory)
        files.push_back(std::filesystem::canonical(elem.path()).c_str());
    std::sort(files.begin(), files.end());
    return files;
  }

  Eigen::Vector3d toHomoCoordVec(const cv::Point2d &pt) {
    return Eigen::Vector3d(pt.x, pt.y, 1.0);
  }

  double homoError(const CBPtsVec &imgPts, const CBPtsVec &objPts, const Eigen::Matrix3d &hMat) {
    std::size_t size = imgPts.size();
    double error = 0.0;
    for (int i = 0; i != size; ++i) {
      auto imgPt = imgPts[i];
      auto objPt = objPts[i];
      Eigen::Vector3d imgPt_t = hMat * toHomoCoordVec(objPt);
      imgPt_t /= imgPt_t(2);
      Eigen::Vector3d delta = toHomoCoordVec(imgPt) - imgPt_t;
      error += std::sqrt(delta.dot(delta));
    }
    return error;
  }
} // namespace ns_st3
