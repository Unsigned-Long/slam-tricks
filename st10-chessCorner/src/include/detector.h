#ifndef DETECTOR_H
#define DETECTOR_H

#include "eigen3/Eigen/Dense"
#include "helper.h"
#include "prototype.h"
#include <tuple>

// #define WRITE_PROC_IMG

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

    using ChessBoard = std::vector<std::vector<std::size_t>>;

  public:
    Detector(ushort protoHWS = 5, ushort nmsHWS = 4, ushort histHWS = 10, ushort refineHWS = 5);

    void solve(cv::Mat gImg);

  protected:
    void compute_likehood();

    void findCorners();

    void verifyCorners();

    void refineCorners();

    void genChessBoard();

  protected:
    std::tuple<bool, float, ChessBoard>
    genChessBoard(std::size_t idx);

    std::pair<bool, ChessBoard>
    initChessBoard(std::size_t idx);

    std::pair<bool, std::size_t>
    closestCornerInDir(std::size_t cornerIdx, const Eigen::Vector2f &dir, float &dist);
  };

} // namespace ns_st10

#endif