#include "help.h"

namespace ns_st13 {
  std::vector<std::string> filesInDir(const std::string &directory) {
    std::vector<std::string> files;
    for (const auto &elem : std::filesystem::directory_iterator(directory))
      if (elem.status().type() != std::filesystem::file_type::directory)
        files.push_back(std::filesystem::canonical(elem.path()).c_str());
    std::sort(files.begin(), files.end());
    return files;
  }

  void showImg(cv::Mat img, const std::string &imgName) {
    cv::namedWindow(imgName, cv::WINDOW_FREERATIO);
    cv::imshow(imgName, img);
    cv::waitKey(0);
    return;
  }
} // namespace ns_st13
