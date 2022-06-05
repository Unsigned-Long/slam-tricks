#include "opencv2/highgui.hpp"
#include "opticalflow.hpp"
#include <filesystem>

std::vector<std::string> filesInDir(const std::string &directory) {
  std::vector<std::string> files;
  for (const auto &elem : std::filesystem::directory_iterator(directory))
    if (elem.status().type() != std::filesystem::file_type::directory)
      files.push_back(std::filesystem::canonical(elem.path()).c_str());
  return files;
}

int main(int argc, char const *argv[]) {
  auto filenames = filesInDir("/home/csl/dataset/rgbd-slam/2022-6-2-8-32-37/color");
  // /home/csl/dataset/rgbd-slam/2022-6-2-8-32-37/color
  // /home/csl/dataset/kitti/sequence/image_0
  std::sort(filenames.begin(), filenames.end());

  auto optFlow = ns_st5::OpticalFlow::create(50, 1);

  for (int i = 0; i != filenames.size(); ++i) {
    const auto &filename = filenames.at(i);
    auto img = cv::imread(filename, cv::IMREAD_UNCHANGED);
    // cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
    auto traceImg = optFlow->track(img).draw();
    cv::imshow("win", traceImg);
    cv::waitKey(10);
  }

  return 0;
}
