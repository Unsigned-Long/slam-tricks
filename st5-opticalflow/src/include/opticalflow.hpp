#ifndef OPTICALFLOW_HPP
#define OPTICALFLOW_HPP

#include "artwork/colorful/color_mapping.hpp"
#include "artwork/logger/logger.h"
#include "opencv2/features2d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/video/tracking.hpp"
#include <list>
#include <random>

namespace ns_st5 {

  struct TrackedPoint {
  public:
    std::list<cv::Point2f> trace;
    cv::Point2f lastPt;
    ushort gr;
    ushort gc;

    TrackedPoint(const cv::Point2f &pt, const ushort gr, const ushort gc)
        : trace(), lastPt(pt), gr(gr), gc(gc) {}
  };

  class OpticalFlow {
  public:
    using Ptr = std::shared_ptr<OpticalFlow>;
    using grid_elem = std::pair<cv::Rect, ushort>;

  private:
    std::default_random_engine engine;
    const ushort GRID_SIZE;
    const ushort PTS_IN_GRID;
    const ushort KEEP_PTS = 30;
    std::vector<std::vector<grid_elem>> _grids;
    std::list<TrackedPoint> _pts;
    cv::Mat _lastGrayImg;
    cv::Mat _lastColorImg;

  public:
    OpticalFlow(const ushort &gridSize, const ushort &ptsInGrid)
        : GRID_SIZE(gridSize), PTS_IN_GRID(ptsInGrid), _grids(), _lastGrayImg(), _pts(), _lastColorImg() {}

    static Ptr create(const ushort &gridSize, const ushort &ptsInGrid) {
      return std::make_shared<OpticalFlow>(gridSize, ptsInGrid);
    }

    OpticalFlow &track(cv::Mat colorImg) {
      this->_lastColorImg = colorImg;
      cv::Mat grayImg;
      cv::cvtColor(colorImg, grayImg, cv::COLOR_BGR2GRAY);
      if (this->_lastGrayImg.empty()) {
        this->_lastGrayImg = grayImg;
        this->initGrids();
        return *this;
      }
      std::vector<cv::Point2f> prevPts, nextPts;

      for (const auto &p : this->_pts) {
        prevPts.push_back(p.lastPt);
      }

      cv::Mat status, err;
      cv::calcOpticalFlowPyrLK(this->_lastGrayImg, grayImg, prevPts, nextPts, status, err);

      auto iter = this->_pts.begin();
      cv::Rect imgRect(0, 0, grayImg.cols, grayImg.rows);
      for (int i = 0; i != nextPts.size(); ++i) {
        uchar statu = status.at<uchar>(i);
        if (statu == 1 && imgRect.contains(nextPts[i]) &&
            distance(iter->lastPt, nextPts[i]) < 0.01f * (grayImg.rows + grayImg.cols)) {
          iter->lastPt = nextPts[i];
          iter->trace.push_front(iter->lastPt);
          if (iter->trace.size() > KEEP_PTS) {
            iter->trace.pop_back();
          }
          ++iter;
        } else {
          --this->_grids[iter->gr][iter->gc].second;
          iter = this->_pts.erase(iter);
        }
      }
      this->_lastGrayImg = grayImg;
      return *this;
    }

    cv::Mat draw() {

      for (const auto &elem : this->_pts) {
        // cv::drawMarker(this->_lastColorImg, elem.lastPt, cv::Scalar(0, 0, 255),
        //                cv::MarkerTypes::MARKER_CROSS, 8, 1);
        if (elem.trace.size() < 2) {
          continue;
        }
        int idx = 0;
        for (auto iter = --elem.trace.cend(); iter != ++elem.trace.cbegin();) {
          auto p1 = *iter;
          auto p2 = *--iter;
          auto rgb = ns_cm::mapping(idx, 0, KEEP_PTS - 2, KEEP_PTS - 1, ns_cm::style::red, true);
          cv::line(_lastColorImg, p1, p2, cv::Scalar(rgb.b, rgb.g, rgb.r));
          ++idx;
        }
      }

      this->addNewPts();
      return this->_lastColorImg;
    }

  protected:
    void initGrids() {
      int rows = this->_lastGrayImg.rows, cols = this->_lastGrayImg.cols;
      const int gr = rows / GRID_SIZE + 1, gc = cols / GRID_SIZE + 1;
      this->_grids.resize(gr);
      for (int r = 0; r != gr; ++r) {
        this->_grids[r].resize(gc);
        for (int c = 0; c != gc; ++c) {
          int orgX = c * GRID_SIZE, orgY = r * GRID_SIZE;
          int spanX = GRID_SIZE, spanY = GRID_SIZE;
          if (r == gr - 1) {
            spanY = rows - orgY;
          }
          if (c == gc - 1) {
            spanX = cols - orgX;
          }
          this->_grids[r][c].first = cv::Rect(orgX, orgY, spanX, spanY);
          // gen random point
          std::uniform_real_distribution<float> ux(0.0f, spanX);
          std::uniform_real_distribution<float> uy(0.0f, spanY);
          for (int i = 0; i != PTS_IN_GRID; ++i) {
            auto p = TrackedPoint(cv::Point2f(orgX + ux(engine), orgY + uy(engine)), r, c);
            this->_pts.push_back(p);
          }
          this->_grids[r][c].second = PTS_IN_GRID;
        }
      }
    }

    void addNewPts() {
      for (auto &elem : this->_pts) {
        if (!this->_grids[elem.gr][elem.gc].first.contains(elem.lastPt)) {
          --this->_grids[elem.gr][elem.gc].second;
          elem.gr = elem.lastPt.y / GRID_SIZE;
          elem.gc = elem.lastPt.x / GRID_SIZE;
          ++this->_grids[elem.gr][elem.gc].second;
        }
      }

      for (int r = 0; r != this->_grids.size(); ++r) {
        for (int c = 0; c != this->_grids[r].size(); ++c) {
          auto &[rect, count] = this->_grids[r][c];
          if (count < PTS_IN_GRID) {
            int toGenPts = PTS_IN_GRID - count;
            std::uniform_real_distribution<float> ux(0.0f, rect.width);
            std::uniform_real_distribution<float> uy(0.0f, rect.height);
            for (int i = 0; i != toGenPts; ++i) {
              auto p = TrackedPoint(cv::Point2f(rect.x + ux(engine), rect.y + uy(engine)), r, c);
              this->_pts.push_back(p);
            }
            this->_grids[r][c].second = PTS_IN_GRID;
          }
        }
      }
    }

    float distance(const cv::Point2f &p1, const cv::Point2f &p2) {
      return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
    }
  };

} // namespace ns_st5

#endif