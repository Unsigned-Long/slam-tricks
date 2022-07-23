#ifndef CHESSBOARD_H
#define CHESSBOARD_H

#include "opencv2/core.hpp"
#include <deque>
#include <set>

namespace ns_st10 {
  struct ChessBoard : public std::deque<std::deque<std::size_t>> {
  public:
    using parent_type = std::deque<std::deque<std::size_t>>;
    // using parents' constructors
    using parent_type::parent_type;

  public:
    // turn the ChessBoard object into the CBCorners object
    CBCorners toCBCorners(const std::vector<cv::Point2f> &corners) const {
      CBCorners cbcs(this->rows(), std::vector<cv::Point2f>(this->cols()));
      // assign
      for (int i = 0; i != this->rows(); ++i) {
        for (int j = 0; j != this->cols(); ++j) {
          cbcs[i][j] = corners[(*this)[i][j]];
        }
      }
      // adjust
      cbcs.adjust();
      return cbcs;
    }

    // put the corners' index into a set
    std::set<std::size_t> toSet() const {
      std::set<std::size_t> s;
      for (int i = 0; i != this->rows(); ++i) {
        for (int j = 0; j != this->cols(); ++j) {
          // insert
          s.insert((*this)[i][j]);
        }
      }
      return s;
    }

    // add current chess board's corner indexes into a existing set
    void addToSet(std::set<std::size_t> &s) const {
      for (int i = 0; i != this->rows(); ++i) {
        for (int j = 0; j != this->cols(); ++j) {
          // insert
          s.insert((*this)[i][j]);
        }
      }
    }

    // the rows
    std::size_t rows() const {
      return this->size();
    }

    // the columns
    std::size_t cols() const {
      return this->front().size();
    }

    // the index of the left Top corner
    std::size_t leftTop() const {
      return (*this)[0][0];
    }

    // the index of the left lower corner
    std::size_t leftLower() const {
      return (*this)[this->rows() - 1][0];
    }

    // the index of the right Top corner
    std::size_t rightTop() const {
      return (*this)[0][this->cols() - 1];
    }

    // the index of the right lower corner
    std::size_t rightLower() const {
      return (*this)[this->rows() - 1][this->cols() - 1];
    }
  };
} // namespace ns_st10

#endif