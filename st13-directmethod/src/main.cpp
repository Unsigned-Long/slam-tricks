#include "direct.h"

int main(int argc, char const *argv[]) {
  std::vector<std::string> imgNames = ns_st13::filesInDir("../img");
  ns_st13::DirectMethod solver;
  for (int i = 0; i != imgNames.size(); ++i) {
    cv::Mat img = cv::imread(imgNames[i], cv::IMREAD_GRAYSCALE);
    ns_st13::showImg(img);
  }
  return 0;
}
