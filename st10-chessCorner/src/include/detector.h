#ifndef DETECTOR_H
#define DETECTOR_H

#include "cbcorners.h"
#include "eigen3/Eigen/Dense"
#include "pcl-1.12/pcl/kdtree/kdtree_flann.h"
#include "pcl-1.12/pcl/point_types.h"
#include "prototype.h"
#include <deque>
#include <tuple>

// #define WRITE_PROC_IMG
#define SHOW_PROC_IMG

namespace ns_st10 {

  class Detector {
  private:
    const ushort PROTO_HWS;
    const ushort NMS_HWS;
    const ushort HIST_HWS;
    const ushort REFINE_HWS;
    cv::Mat grayImg;
    cv::Mat likehood;
    cv::Mat gradX;
    cv::Mat gradY;
    std::vector<cv::Point> corners;

    std::vector<cv::Point2f> corners_sp;
    std::vector<float> scores;
    std::vector<std::pair<float, float>> corners_modes;
    pcl::KdTreeFLANN<pcl::PointXY>::Ptr kdtree;

  public:
    using ChessBoard = std::deque<std::deque<std::size_t>>;

    enum class CBGrowDir {
      TOP,
      LOWER,
      LEFT,
      RIGHT
    };

  public:
    Detector(ushort protoHWS = 10, ushort nmsHWS = 4, ushort histHWS = 10, ushort refineHWS = 5);

    std::pair<bool, CBCorners> solve(cv::Mat gImg, bool computeEach = false);

  protected:
    void compute_likehood();

    void findCorners();

    void verifyCorners();

    void refineCorners();

    std::pair<bool, CBCorners> genChessBoard(bool computeEach);

  protected:
    std::tuple<bool, float, ChessBoard>
    genChessBoardOnce(std::size_t idx);

    std::pair<bool, ChessBoard>
    initChessBoard(std::size_t idx);

    std::pair<bool, std::size_t>
    closestCornerInDir(std::size_t cornerIdx, const Eigen::Vector2f &dir, float &dist);

    float chessBoardEnergy(const ChessBoard &cb);

    std::pair<float, ChessBoard>
    growChessBoard(const ChessBoard &cb, CBGrowDir dir);

    cv::Point2f predictCorner(const cv::Point2f &p1, const cv::Point2f &p2);
  };

} // namespace ns_st10

#endif