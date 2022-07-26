#ifndef CBCORNER_H
#define CBCORNER_H

#include "opencv2/core.hpp"

namespace ns_st3 {
  struct CBPtsVec : public std::vector<cv::Point2d> {
  public:
    using parent_type = std::vector<cv::Point2d>;
    // using parents' constructors
    using parent_type::parent_type;
  };

  struct CBCorners : public std::vector<std::vector<cv::Point2d>> {
  public:
    using parent_type = std::vector<std::vector<cv::Point2d>>;
    // using parents' constructors
    using parent_type::parent_type;

  public:
    // adjust the order of current chess board [make it a right-hand coordinate system]
    void adjust();

    // write the chess board to file
    void write(const std::string &filename);

    // load chess board from file
    static CBCorners read(const std::string &filename);

    // rows
    std::size_t rows() const {
      return this->size();
    }

    // cosl
    std::size_t cols() const {
      return this->front().size();
    }
  };
} // namespace ns_st10

#endif