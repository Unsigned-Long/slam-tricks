#ifndef DETECTOR_H
#define DETECTOR_H

#include "eigen3/Eigen/Dense"
#include "helper.h"
#include "pcl-1.12/pcl/kdtree/kdtree_flann.h"
#include "pcl-1.12/pcl/point_types.h"
#include "prototype.h"
#include <deque>
#include <tuple>

// macro for saving the image during processing
// #define WRITE_PROC_IMG

// macro for displaying the image during processing
#define SHOW_PROC_IMG

// macro for display the growing chessboard during processing
// #define SHOW_GROW_CB

// macro for timing during processing
#define TIMING_PROC

namespace ns_st10 {

  class Detector {
  private:
    // the half window size for prototypes
    const ushort PROTO_HWS;
    // the half window size for non maximum suppression algorithm
    const ushort NMS_HWS;
    // the half window size for histogram of statistical pixel gradient
    const ushort HIST_HWS;
    // the half window size for refine corners
    const ushort REFINE_HWS;
    // the gray image
    cv::Mat grayImg;
    // the likehood image
    cv::Mat likehood;
    // the grad image in x-dir
    cv::Mat gradX;
    // the grad image in y-dir
    cv::Mat gradY;
    // the corners [Integer pixel level]
    std::vector<cv::Point> corners;

    // the corners [Subpixel level]
    std::vector<cv::Point2f> corners_sp;
    // corners' scores
    std::vector<float> scores;
    // corners' modes
    std::vector<std::pair<float, float>> corners_modes;
    // the kdtree for fast search
    pcl::KdTreeFLANN<pcl::PointXY>::Ptr kdtree;

  public:
    // chessboard growth directions
    enum class CBGrowDir {
      TOP,
      LOWER,
      LEFT,
      RIGHT
    };

  public:
    // constructor
    Detector(ushort protoHWS = 10, ushort nmsHWS = 4, ushort histHWS = 10, ushort refineHWS = 5);

    /**
     * @brief the main function
     *
     * @param gImg the gray image
     * @param computeEach whether grow chessboard u each corners
     * @return std::pair<bool, CBCorners>
     */
    std::pair<bool, CBCorners> solve(cv::Mat gImg, bool computeEach = false);

    std::vector<CBCorners> solveMutiCB(cv::Mat gImg);

  protected:
    void compute_likehood();

    void findCorners();

    void verifyCorners();

    void refineCorners();

    std::pair<bool, CBCorners> genChessBoard(bool computeEach);

    std::vector<CBCorners> genChessBoard();

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