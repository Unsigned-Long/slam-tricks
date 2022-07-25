#include "helper.h"
#include <cmath>

namespace ns_st10 {
  cv::Mat cvt_32FC1_8UC1(cv::Mat img_f) {
    // find min and max values
    double min, max;
    cv::minMaxIdx(img_f, &min, &max);
    cv::Mat gray(img_f.size(), CV_8UC1);
    // 255*(pixel - min)/(max - min)
    float factor = 255.0 / (max - min);
    // convert
    img_f.convertTo(gray, CV_8UC1, factor, -min * factor);
    return gray;
  }

  float gauss(float x, float mean, float sigma) {
    const float sqrt_2pi = std::sqrt(2.0f * M_PI);
    const float index = -(x - mean) * (x - mean) / (2.0f * sigma * sigma);
    return 1.0 / (sqrt_2pi * sigma) * std::exp(index);
  }

  void showImg(cv::Mat img, const std::string &name) {
    cv::namedWindow(name, cv::WINDOW_FREERATIO);
    cv::imshow(name, img);
    return;
  }

  bool inAngleRange(float angle, float startAng, float endAng) {
    // make sure each angle is a valid angle
    if (angle < 0.0f) {
      angle += 2.0 * M_PI;
    } else if (angle > 2.0 * M_PI) {
      angle -= 2.0 * M_PI;
    }
    if (startAng < 0.0f) {
      startAng += 2.0 * M_PI;
    } else if (startAng > 2.0 * M_PI) {
      startAng -= 2.0 * M_PI;
    }
    if (endAng < 0.0f) {
      endAng += 2.0 * M_PI;
    } else if (endAng > 2.0 * M_PI) {
      endAng -= 2.0 * M_PI;
    }
    // return result according to the size of the two angles
    if (startAng < endAng) {
      return angle > startAng && angle < endAng;
    } else {
      return angle > startAng || angle < endAng;
    }
  }

  std::vector<cv::Point2i> nms2d(const cv::Mat img, const ushort hws) {
    std::vector<cv::Point2i> max;
    int rows = img.rows, cols = img.cols;
    ushort ws = 2 * hws + 1;
    for (int i = 0; i < rows; i += hws + 1) {
      for (int j = 0; j < cols; j += hws + 1) {
        // find max val and idx
        float maxVal;
        cv::Point2i maxIdx;
        {
          cv::Mat win = img(cv::Range(i, std::min(i + hws + 1, rows)), cv::Range(j, std::min(j + hws + 1, cols)));
          double maxVal_t;
          int maxIdx_t[2];
          cv::minMaxIdx(win, nullptr, &maxVal_t, nullptr, maxIdx_t);
          maxVal = maxVal_t;
          maxIdx.y = maxIdx_t[0] + i;
          maxIdx.x = maxIdx_t[1] + j;
        }

        bool isMax = true;
        for (int r = std::max(0, maxIdx.y - hws); r < std::min(maxIdx.y + hws + 1, rows); ++r) {
          for (int c = std::max(0, maxIdx.x - hws); c < std::min(maxIdx.x + hws + 1, cols); ++c) {
            if (r == maxIdx.y && c == maxIdx.x) {
              continue;
            }
            float val = img.at<float>(r, c);
            if (val > maxVal) {
              isMax = false;
              break;
            }
          }
          if (!isMax) {
            break;
          }
        }
        if (isMax) {
          max.push_back(maxIdx);
        }
      }
    }
    return max;
  }

  cv::Mat drawMarks(cv::Mat grayImg, const std::vector<cv::Point> &pts) {
    cv::Mat color;
    cv::cvtColor(grayImg, color, cv::COLOR_GRAY2BGR);
    int size = grayImg.cols / 50;
    for (const auto &pt : pts) {
      cv::drawMarker(color, pt, cv::Scalar(0, 0, 255), cv::MARKER_CROSS, size, size * 0.2);
    }
    return color;
  }

  cv::Mat drawMarks(cv::Mat grayImg, const std::vector<cv::Point2f> &pts) {
    cv::Mat color;
    cv::cvtColor(grayImg, color, cv::COLOR_GRAY2BGR);
    int size = grayImg.cols / 50;
    for (const auto &pt : pts) {
      cv::drawMarker(color, pt, cv::Scalar(0, 0, 255), cv::MARKER_CROSS, size, size * 0.2);
    }
    return color;
  }

  cv::Mat drawModes(cv::Mat grayImg, const std::vector<cv::Point2f> &pts,
                    const std::vector<std::pair<float, float>> &modes) {
    cv::Mat color;
    cv::cvtColor(grayImg, color, cv::COLOR_GRAY2BGR);
    int size1 = grayImg.cols / 40;
    int size2 = grayImg.cols / 300;
    for (int i = 0; i != pts.size(); ++i) {
      const auto &pt = pts[i];
      const auto &m = modes[i];
      float alpha1 = m.first, alpha2 = m.second;
      {
        float vx = std::cos(alpha1) * size1, vy = std::sin(alpha1) * size1;
        cv::Point2f p(pt.x + vx, pt.y + vy);
        cv::line(color, pt, p, cv::Scalar(0, 255, 0), size2);
      }
      {
        float vx = std::cos(alpha2) * size1, vy = std::sin(alpha2) * size1;
        cv::Point2f p(pt.x + vx, pt.y + vy);
        cv::line(color, pt, p, cv::Scalar(255, 0, 0), size2);
      }
    }
    return color;
  }

  cv::Mat drawModes(cv::Mat grayImg, const std::vector<cv::Point> &pts,
                    const std::vector<std::pair<float, float>> &modes) {
    cv::Mat color;
    cv::cvtColor(grayImg, color, cv::COLOR_GRAY2BGR);
    int size1 = grayImg.cols / 40;
    int size2 = grayImg.cols / 300;
    for (int i = 0; i != pts.size(); ++i) {
      const auto &pt = pts[i];
      const auto &m = modes[i];
      float alpha1 = m.first, alpha2 = m.second;
      {
        float vx = std::cos(alpha1) * size1, vy = std::sin(alpha1) * size1;
        cv::Point2f p(pt.x + vx, pt.y + vy);
        cv::line(color, pt, p, cv::Scalar(0, 255, 0), size2);
      }
      {
        float vx = std::cos(alpha2) * size1, vy = std::sin(alpha2) * size1;
        cv::Point2f p(pt.x + vx, pt.y + vy);
        cv::line(color, pt, p, cv::Scalar(255, 0, 0), size2);
      }
    }
    return color;
  }

  void gaussFilter(std::vector<float> &ary) {
    const float a1 = 0.1f, a2 = 0.2f, a3 = 0.4f, a4 = 0.2f, a5 = 0.1f;
    int size = ary.size();
    std::vector<float> res(ary.size());
    for (int i = 0; i != ary.size(); ++i) {
      res[i] = a1 * ary[((i - 2) + size) % size] +
               a2 * ary[((i - 1) + size) % size] +
               a3 * ary[((i) + size) % size] +
               a4 * ary[((i + 1) + size) % size] +
               a5 * ary[((i + 2) + size) % size];
    }
    ary = res;
  }

  std::pair<std::size_t, std::size_t>
  meanShift(const std::vector<float> &ary) {
    float idx1 = 6.0f, idx2 = 16.0f, idx3 = 26.0f;
    const std::size_t size = ary.size();

    auto weightFunc = [&ary, &size](std::size_t idx, std::size_t halfWinSize) -> float {
      float vec = 0.0f, total = 0.0f;
      for (int i = idx - halfWinSize; i != idx + halfWinSize + 1; ++i) {
        float val = ary[(i + size) % size];
        vec += (i - float(idx)) * val;
        total += val;
      }
      vec /= total;
      return vec;
    };

    for (int i = 0; i != 10; ++i) {
      idx1 += weightFunc((int(idx1 + 0.5f) + size) % size, 4);
      idx2 += weightFunc((int(idx2 + 0.5f) + size) % size, 4);
      idx3 += weightFunc((int(idx3 + 0.5f) + size) % size, 4);
    }

    for (int i = 0; i != 10; ++i) {
      idx1 += weightFunc((int(idx1 + 0.5f) + size) % size, 2);
      idx2 += weightFunc((int(idx2 + 0.5f) + size) % size, 2);
      idx3 += weightFunc((int(idx3 + 0.5f) + size) % size, 2);
    }

    auto choiceFunc = [&size, &ary](std::size_t idx) -> std::size_t {
      // get integer index
      int lower = (int(idx) + size) % size;
      int top = (int(idx + 0.5f) + size) % size;
      int mid;
      if (ary[lower] > ary[top]) {
        mid = lower;
      } else {
        mid = top;
      }
      // get the maximum index
      lower = (mid - 1) % size;
      top = (mid + 1) % size;
      if (ary[lower] > ary[mid] && ary[lower] >= ary[top]) {
        return lower;
      } else if (ary[mid] > ary[lower] && ary[mid] >= ary[top]) {
        return mid;
      } else {
        return top;
      }
    };
    int i1 = choiceFunc(idx1);
    int i2 = choiceFunc(idx2);
    int i3 = choiceFunc(idx3);

    // LOG_VAR(i1, i2, i3);
    // LOG_VAR(idx1, idx2, idx3);

    int dis12 = std::abs(i1 - i2), dis13 = std::abs(i1 - i3), dis23 = std::abs(i2 - i3);
    if (dis12 > 16) {
      dis12 = 32 - dis12;
    }
    if (dis13 > 16) {
      dis13 = 32 - dis13;
    }
    if (dis23 > 16) {
      dis23 = 32 - dis23;
    }
    int min = std::min({dis12, dis13, dis23});
    if (dis12 == min) {
      return {i3, ary[i1] > ary[i2] ? i1 : i2};
    } else if (dis13 == min) {
      return {i2, ary[i1] > ary[i3] ? i1 : i3};
    } else {
      return {i1, ary[i2] > ary[i3] ? i2 : i3};
    }
  }

  cv::Mat drawChessBoard(cv::Mat grayImg, const ChessBoard &board,
                         const std::vector<cv::Point2f> &corner) {
    cv::Mat color;
    cv::cvtColor(grayImg, color, cv::COLOR_GRAY2BGR);
    int size1 = grayImg.cols / 100;
    int size2 = grayImg.cols / 300;
    std::size_t rows = board.size();
    std::size_t cols = board.front().size();

    for (int i = 0; i != rows; ++i) {
      for (int j = 0; j != cols - 1; ++j) {
        auto &cp = corner[board[i][j]];
        auto &rp = corner[board[i][j + 1]];
        cv::line(color, cp, rp, cv::Scalar(0, 0, 255), size1);
      }
    }
    for (int i = 0; i != rows - 1; ++i) {
      for (int j = 0; j != cols; ++j) {
        auto &cp = corner[board[i][j]];
        auto &lp = corner[board[i + 1][j]];
        cv::line(color, cp, lp, cv::Scalar(0, 0, 255), size1);
      }
    }
    for (int i = 0; i != rows; ++i) {
      for (int j = 0; j != cols - 1; ++j) {
        auto &cp = corner[board[i][j]];
        auto &rp = corner[board[i][j + 1]];
        cv::line(color, cp, rp, cv::Scalar(255, 255, 255), size2);
      }
    }
    for (int i = 0; i != rows - 1; ++i) {
      for (int j = 0; j != cols; ++j) {
        auto &cp = corner[board[i][j]];
        auto &lp = corner[board[i + 1][j]];
        cv::line(color, cp, lp, cv::Scalar(255, 255, 255), size2);
      }
    }

    cv::Point2f op = corner[board[0][0]];
    cv::Point2f xp = corner[board[0][cols - 1]];
    cv::Point2f yp = corner[board[rows - 1][0]];
    cv::circle(color, op, size1, cv::Scalar(0, 0, 255), size1 * 2);
    cv::circle(color, xp, size1, cv::Scalar(0, 255, 0), size1 * 2);
    cv::circle(color, yp, size1, cv::Scalar(255, 0, 0), size1 * 2);

    return color;
  }

  void drawChessBoard_h(cv::Mat color, const CBCorners &board) {
    int size1 = color.cols / 100;
    int size2 = color.cols / 300;
    std::size_t rows = board.size();
    std::size_t cols = board.front().size();

    for (int i = 0; i != rows; ++i) {
      for (int j = 0; j != cols - 1; ++j) {
        auto &cp = board[i][j];
        auto &rp = board[i][j + 1];
        cv::line(color, cp, rp, cv::Scalar(0, 0, 255), size1);
      }
    }
    for (int i = 0; i != rows - 1; ++i) {
      for (int j = 0; j != cols; ++j) {
        auto &cp = board[i][j];
        auto &lp = board[i + 1][j];
        cv::line(color, cp, lp, cv::Scalar(0, 0, 255), size1);
      }
    }
    for (int i = 0; i != rows; ++i) {
      for (int j = 0; j != cols - 1; ++j) {
        auto &cp = board[i][j];
        auto &rp = board[i][j + 1];
        cv::line(color, cp, rp, cv::Scalar(255, 255, 255), size2);
      }
    }
    for (int i = 0; i != rows - 1; ++i) {
      for (int j = 0; j != cols; ++j) {
        auto &cp = board[i][j];
        auto &lp = board[i + 1][j];
        cv::line(color, cp, lp, cv::Scalar(255, 255, 255), size2);
      }
    }

    cv::Point2f op = board[0][0];
    cv::Point2f xp = board[0][cols - 1];
    cv::Point2f yp = board[rows - 1][0];
    // red [O]
    cv::circle(color, op, size1, cv::Scalar(0, 0, 255), size1 * 2);
    // green [X]
    cv::circle(color, xp, size1, cv::Scalar(0, 255, 0), size1 * 2);
    // blue [Y]
    cv::circle(color, yp, size1, cv::Scalar(255, 0, 0), size1 * 2);
  }

  cv::Mat drawChessBoard(cv::Mat grayImg, const CBCorners &board) {
    cv::Mat color;
    cv::cvtColor(grayImg, color, cv::COLOR_GRAY2BGR);
    drawChessBoard_h(color, board);
    return color;
  }

  cv::Mat drawChessBoard(cv::Mat grayImg, const std::vector<CBCorners> &boards) {
    cv::Mat color;
    cv::cvtColor(grayImg, color, cv::COLOR_GRAY2BGR);
    for (const auto &board : boards) {
      drawChessBoard_h(color, board);
    }
    return color;
  }
} // namespace ns_st10
