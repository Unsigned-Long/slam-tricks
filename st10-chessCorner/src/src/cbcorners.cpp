#include "cbcorners.h"
#include "eigen3/Eigen/Dense"

namespace ns_st10 {
  void CBCorners::adjust() {
    auto &board = *this;
    std::size_t rows = board.size();
    std::size_t cols = board.front().size();
    cv::Point2f op = board[0][0];
    cv::Point2f xp = board[0][cols - 1];
    cv::Point2f yp = board[rows - 1][0];
    Eigen::Vector3f ox(xp.x - op.x, xp.y - op.y, 0.0f);
    Eigen::Vector3f oy(yp.x - op.x, yp.y - op.y, 0.0f);
    float dir = ox.cross(oy)[2];
    LOG_VAR(dir);
    if (dir > 0.0f) {
      return;
    }
    // adjust
    CBCorners newCB(cols, std::vector<cv::Point2f>(rows));
    for (int i = 0; i != rows; ++i) {
      for (int j = 0; j != cols; ++j) {
        newCB[j][i] = board[i][j];
      }
    }
    board = newCB;
  }
  void CBCorners::write(const std::string &filename) {
    std::size_t rows = this->size();
    std::size_t cols = this->front().size();
    std::fstream file(filename, std::ios::out);
    file << "#" << rows << ',' << cols << std::endl;
    for (int i = 0; i != rows; ++i) {
      for (int j = 0; j != cols; ++j) {
        file << i << ',' << j << ',' << (*this)[i][j].x << ',' << (*this)[i][j].y << std::endl;
      }
    }
    file.close();
  }
} // namespace ns_st10
