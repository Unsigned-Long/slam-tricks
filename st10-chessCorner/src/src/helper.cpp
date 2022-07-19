#include "helper.h"
#include <cmath>

namespace ns_st10 {
  cv::Mat cvt_32FC1_8UC1(cv::Mat img_f) {
    double min, max;
    cv::minMaxIdx(img_f, &min, &max);
    cv::Mat gray(img_f.size(), CV_8UC1);
    // 255*(pixel - min)/(max - min)
    float factor = 255.0 / (max - min);
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
    for (const auto &pt : pts) {
      cv::drawMarker(color, pt, cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 10);
    }
    return color;
  }

  cv::Mat drawMarks(cv::Mat grayImg, const std::vector<cv::Point2f> &pts) {
    cv::Mat color;
    cv::cvtColor(grayImg, color, cv::COLOR_GRAY2BGR);
    for (const auto &pt : pts) {
      cv::drawMarker(color, pt, cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 10);
    }
    return color;
  }

  cv::Mat drawModes(cv::Mat grayImg, const std::vector<cv::Point2f> &pts,
                    const std::vector<std::pair<float, float>> &modes) {
    cv::Mat color;
    cv::cvtColor(grayImg, color, cv::COLOR_GRAY2BGR);
    for (int i = 0; i != pts.size(); ++i) {
      const auto &pt = pts[i];
      const auto &m = modes[i];
      float alpha1 = m.first, alpha2 = m.second;
      {
        float vx = std::cos(alpha1) * 20, vy = std::sin(alpha1) * 20;
        cv::Point2f p(pt.x + vx, pt.y + vy);
        cv::line(color, pt, p, cv::Scalar(0, 255, 0), 2);
      }
      {
        float vx = std::cos(alpha2) * 20, vy = std::sin(alpha2) * 20;
        cv::Point2f p(pt.x + vx, pt.y + vy);
        cv::line(color, pt, p, cv::Scalar(255, 0, 0), 2);
      }
    }
    return color;
  }

  cv::Mat drawModes(cv::Mat grayImg, const std::vector<cv::Point> &pts,
                    const std::vector<std::pair<float, float>> &modes) {
    cv::Mat color;
    cv::cvtColor(grayImg, color, cv::COLOR_GRAY2BGR);
    for (int i = 0; i != pts.size(); ++i) {
      const auto &pt = pts[i];
      const auto &m = modes[i];
      float alpha1 = m.first, alpha2 = m.second;
      {
        float vx = std::cos(alpha1) * 20, vy = std::sin(alpha1) * 20;
        cv::Point2f p(pt.x + vx, pt.y + vy);
        cv::line(color, pt, p, cv::Scalar(0, 255, 0), 2);
      }
      {
        float vx = std::cos(alpha2) * 20, vy = std::sin(alpha2) * 20;
        cv::Point2f p(pt.x + vx, pt.y + vy);
        cv::line(color, pt, p, cv::Scalar(255, 0, 0), 2);
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

  std::pair<std::size_t, std::size_t> meanShift(const std::vector<float> &ary) {
    float idx1 = 6.0f, idx2 = 16.0f, idx3 = 26.0f;
    const std::size_t hws = 4, size = ary.size();

    auto weightFunc = [&ary, &hws, &size](std::size_t idx) -> float {
      float vec = 0.0f, total = 0.0f;
      for (int i = idx - hws; i != idx + hws + 1; ++i) {
        float val = ary[(i + size) % size];
        vec += (i - float(idx)) * val;
        total += val;
      }
      vec /= total;
      return vec;
    };

    for (int i = 0; i != 10; ++i) {
      idx1 += weightFunc((int(idx1 + 0.5f) + size) % size);
      idx2 += weightFunc((int(idx2 + 0.5f) + size) % size);
      idx3 += weightFunc((int(idx3 + 0.5f) + size) % size);
    }
    auto choiceFunc = [&size, &ary](std::size_t idx) -> std::size_t {
      int lower = (int(idx) + size) % size;
      int top = (int(idx + 0.5f) + size) % size;
      if (ary[lower] > ary[top]) {
        return lower;
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

  cv::Mat drawChessBoard(cv::Mat grayImg,
                         const std::deque<std::deque<std::size_t>> &board,
                         const std::vector<cv::Point2f> &corner) {
    cv::Mat color;
    cv::cvtColor(grayImg, color, cv::COLOR_GRAY2BGR);
    for (int i = 0; i != board.size(); ++i) {
      for (int j = 0; j != board[0].size(); ++j) {
        const cv::Point2f &pt = corner[board[i][j]];
        cv::drawMarker(color, pt, cv::Scalar(0, 255, 0), cv::MARKER_SQUARE, 10);
        cv::putText(color, std::to_string(i) + ',' + std::to_string(j), pt,
                    cv::HersheyFonts::FONT_HERSHEY_COMPLEX_SMALL, 0.7f, cv::Scalar(0, 0, 255), 1);
      }
    }
    return color;
  }
  cv::Mat drawChessBoard(cv::Mat grayImg,
                         const std::vector<std::vector<cv::Point2f>> &board) {
    cv::Mat color;
    cv::cvtColor(grayImg, color, cv::COLOR_GRAY2BGR);
    for (int i = 0; i != board.size(); ++i) {
      for (int j = 0; j != board[0].size(); ++j) {
        const cv::Point2f &pt = board[i][j];
        cv::drawMarker(color, pt, cv::Scalar(0, 255, 0), cv::MARKER_SQUARE, 10);
        cv::putText(color, std::to_string(i) + ',' + std::to_string(j), pt,
                    cv::HersheyFonts::FONT_HERSHEY_COMPLEX_SMALL, 0.7f, cv::Scalar(0, 0, 255), 1);
      }
    }
    return color;
  }
} // namespace ns_st10
