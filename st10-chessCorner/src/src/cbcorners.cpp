#include "cbcorners.h"
#include "eigen3/Eigen/Dense"
#include <fstream>
#include <iomanip>

namespace ns_st10 {
  void CBCorners::adjust() {
    auto &board = *this;
    std::size_t rows = board.rows();
    std::size_t cols = board.cols();
    cv::Point2f op = board[0][0];
    cv::Point2f xp = board[0][cols - 1];
    cv::Point2f yp = board[rows - 1][0];
    // construct two vectors: ox and oy
    Eigen::Vector3f ox(xp.x - op.x, xp.y - op.y, 0.0f);
    Eigen::Vector3f oy(yp.x - op.x, yp.y - op.y, 0.0f);
    // The sign of the third component of the cross product vector determines the coordinate system
    float dir = ox.cross(oy)[2];
    if (dir > 0.0f) {
      // is a right-hand coordinate system
      return;
    }
    // a left-hand coordinate system, adjust it [transpose]
    CBCorners newCB(cols, std::vector<cv::Point2f>(rows));
    for (int i = 0; i != rows; ++i) {
      for (int j = 0; j != cols; ++j) {
        newCB[j][i] = board[i][j];
      }
    }
    // assign
    board = newCB;
  }

  void CBCorners::write(const std::string &filename) {
    std::size_t rows = this->rows();
    std::size_t cols = this->cols();
    std::fstream file(filename, std::ios::out);
    // write the rows and cols for chess board
    file << rows << ',' << cols << std::endl;
    file << std::fixed << std::setprecision(3);
    // write each chess board corner
    for (int i = 0; i != rows; ++i) {
      for (int j = 0; j != cols; ++j) {
        file << i << ',' << j << ',' << (*this)[i][j].x << ',' << (*this)[i][j].y << std::endl;
      }
    }
    file.close();
  }

  CBCorners CBCorners::read(const std::string &filename) {
    // the ifstream
    std::fstream file(filename, std::ios::in);
    std::string str;
    // get rows and cols for the chess board
    std::getline(file, str);
    std::size_t pos = str.find_first_of(',');
    std::size_t rows = std::stoi(str.substr(0, pos));
    std::size_t cols = std::stoi(str.substr(pos + 1));
    CBCorners cb(rows, std::vector<cv::Point2f>(cols));
    // read each chess board corner
    while (std::getline(file, str)) {
      auto p1 = str.find_first_of(',');
      std::size_t r = std::stoi(str.substr(0, p1));
      auto p2 = str.find_first_of(',', p1 + 1);
      std::size_t c = std::stoi(str.substr(p1 + 1, p2 - p1 - 1));
      auto p3 = str.find_first_of(',', p2 + 1);
      float x = std::stof(str.substr(p2 + 1, p3 - p2 - 1));
      float y = std::stof(str.substr(p3 + 1));
      cb[r][c] = cv::Point2f(x, y);
    }
    file.close();
    return cb;
  }
} // namespace ns_st10
