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
// #define SHOW_PROC_IMG

// macro for display the growing chessboard during processing
// #define SHOW_GROW_CB

// macro for timing during processing
#define TIMING_PROC

namespace ns_st10 {

  class Detector {
  private:
    // the half window size for prototypes
    const ushort PROTO_HWS;
    // the half window size for histogram of statistical pixel gradient
    const ushort HIST_HWS;
    // the half window size for refine corners
    const ushort REFINE_HWS;
    const float FIND_CORNERS_THD;
    const float VERIFY_CORNERS_THD;

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
    Detector(float findCornersThd = 0.4f, float verifyCornersThd = 0.4f,
             ushort protoHWS = 10, ushort histHWS = 10, ushort refineHWS = 5);

    /**
     * @brief the main function to detect chess board in an image
     *
     * @param gImg the gray image
     * @param computeEach whether grow chessboard u each corners
     * @return std::pair<bool, CBCorners> false means no valid chess board detected
     */
    std::pair<bool, CBCorners> solve(cv::Mat gImg, bool computeEach = false);

    /**
     * @brief the main function to detect multi chess boards in an image
     *
     * @param gImg the gray image
     * @return std::vector<CBCorners> the chess boards, empty vector means no valid chess board detected
     */
    std::vector<CBCorners> solveMutiCB(cv::Mat gImg);

  protected:
    /**
     * @brief compute the likehood image for the input gray image
     */
    void compute_likehood();

    /**
     * @brief find possible corners according to the likehood image
     */
    void findCorners();

    /**
     * @brief verify the chosen corners [according to the modes and total score of the corners]
     */
    void verifyCorners();

    /**
     * @brief make the integer pixel into float pixel [subpixel] and make modes more exact
     */
    void refineCorners();

    /**
     * @brief using the left corners to grow the chess board [used in the 'solve' function]
     *
     * @param computeEach whether grow chess board for each corner
     * @return std::pair<bool, CBCorners>
     */
    std::pair<bool, CBCorners> genChessBoard(bool computeEach);

    /**
     * @brief using the left corners to grow multi chess boards [used in the 'solveMutiCB' function]
     *
     * @return std::vector<CBCorners>
     */
    std::vector<CBCorners> genChessBoard();

  protected:
    /**
     * @brief grow chess board for a corner
     *
     * @param idx the index of the corner
     * @return std::tuple<bool, float, ChessBoard> [false means no valid chess board, energy, chess board]
     */
    std::tuple<bool, float, ChessBoard>
    genChessBoardOnce(std::size_t idx);

    /**
     * @brief initialize a 3x3 chess board for a corner
     *
     * @param idx the index of the corner
     * @return std::pair<bool, ChessBoard> [false means initalization is failed]
     */
    std::pair<bool, ChessBoard>
    initChessBoard(std::size_t idx);

    /**
     * @brief find the closest corner from current corner in a certain direction
     *
     * @param cornerIdx the index of current corner
     * @param dir the dirction [Vector2f]
     * @param dist the distance of two corners
     * @return std::pair<bool, std::size_t> [false means no corner found, the closest corner's index]
     */
    std::pair<bool, std::size_t>
    closestCornerInDir(std::size_t cornerIdx, const Eigen::Vector2f &dir, float &dist);

    /**
     * @brief compute the chess board's energy
     *
     * @param cb the chess board
     * @return float the energy
     */
    float chessBoardEnergy(const ChessBoard &cb);

    /**
     * @brief grow chess board in a certain dirction
     *
     * @param cb the chess board
     * @param dir the direction [Top, Lower, Left, Right]
     * @return std::pair<float, ChessBoard> [energy, chess board]
     */
    std::pair<float, ChessBoard>
    growChessBoard(const ChessBoard &cb, CBGrowDir dir);

    /**
     * @brief predict the corner
     *
     * @param p1 the first corner
     * @param p2 the second corner
     * @return cv::Point2f the predicted third corner
     */
    cv::Point2f predictCorner(const cv::Point2f &p1, const cv::Point2f &p2);

    void destoryHistory();
  };

} // namespace ns_st10

#endif