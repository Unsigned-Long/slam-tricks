#include "detector.h"
#include <filesystem>

std::vector<std::string> filesInDir(const std::string &directory) {
  std::vector<std::string> files;
  for (const auto &elem : std::filesystem::directory_iterator(directory))
    if (elem.status().type() != std::filesystem::file_type::directory)
      files.push_back(std::filesystem::canonical(elem.path()).c_str());
  std::sort(files.begin(), files.end());
  return files;
}

int main(int argc, char const *argv[]) {
  auto solver = ns_st10::Detector();
  auto imgs = filesInDir("../img/clib");
  for (const auto &imgPath : imgs) {
    cv::Mat img = cv::imread(imgPath);
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    auto res = solver.solve(img, true);
    if (res.first) {
      auto p1 = imgPath.find_last_of('/');
      auto p2 = imgPath.find_last_of('.');
      res.second.write("../output/clib/" + imgPath.substr(p1 + 1, p2 - p1 - 1) + ".txt");
    }
    cv::destroyAllWindows();
  }
  return 0;
}
