#ifndef CBCORNER_H
#define CBCORNER_H

#include "helper.h"

namespace ns_st10 {

  struct CBCorners : public std::vector<std::vector<cv::Point2f>> {
  public:
    using parent_type = std::vector<std::vector<cv::Point2f>>;
    using parent_type::parent_type;

  public:
    void adjust();
  };
} // namespace ns_st10

#endif