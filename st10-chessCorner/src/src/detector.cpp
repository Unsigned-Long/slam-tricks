#include "detector.h"
#include "artwork/timer/timer.h"
#include <set>

namespace ns_st10 {
  Detector::Detector(float findCornersThd, float verifyCornersThd,
                     ushort protoHWS, ushort nmsHWS,
                     ushort histHWS, ushort refineHWS)
      : FIND_CORNERS_THD(findCornersThd), VERIFY_CORNERS_THD(verifyCornersThd),
        PROTO_HWS(protoHWS), NMS_HWS(nmsHWS),
        HIST_HWS(histHWS), REFINE_HWS(refineHWS) {}

  std::pair<bool, CBCorners>
  Detector::solve(cv::Mat gImg, bool computeEach) {
    // assign
    this->grayImg = gImg;
#ifdef TIMING_PROC
    // the timer
    ns_timer::Timer timer;
    timer.re_start();
#endif
    // compute the likehood image
    compute_likehood();
#ifdef TIMING_PROC
    LOG_INFO(timer.last_elapsed("compute_likehood"));
#endif

#ifdef WRITE_PROC_IMG
    cv::imwrite("../img/process/likehood.png", cvt_32FC1_8UC1(likehood));
#endif
#ifdef SHOW_PROC_IMG
    showImg(cvt_32FC1_8UC1(likehood), "likehood");
    cv::waitKey(0);
#endif

#ifdef TIMING_PROC
    timer.re_start();
#endif
    // find possible corners
    findCorners();
#ifdef TIMING_PROC
    LOG_INFO(timer.last_elapsed("findCorners"));
#endif

#ifdef WRITE_PROC_IMG
    cv::imwrite("../img/process/findCorners.png", drawMarks(grayImg, corners));
#endif
#ifdef SHOW_PROC_IMG
    showImg(drawMarks(grayImg, corners), "findCorners");
    cv::waitKey(0);
#endif

#ifdef TIMING_PROC
    timer.re_start();
#endif
    // verify the corners
    verifyCorners();
#ifdef TIMING_PROC
    LOG_INFO(timer.last_elapsed("verifyCorners"));
#endif

#ifdef WRITE_PROC_IMG
    cv::imwrite("../img/process/oldCorners.png", drawMarks(grayImg, corners));
    cv::imwrite("../img/process/oldModes.png", drawModes(grayImg, corners, corners_modes));
#endif
#ifdef SHOW_PROC_IMG
    showImg(drawMarks(grayImg, corners), "oldCorners");
    showImg(drawModes(grayImg, corners, corners_modes), "oldModes");
    cv::waitKey(0);
#endif

#ifdef TIMING_PROC
    timer.re_start();
#endif
    // refine the corners
    refineCorners();
#ifdef TIMING_PROC
    LOG_INFO(timer.last_elapsed("refineCorners"));
#endif

#ifdef WRITE_PROC_IMG
    cv::imwrite("../img/process/newCorners.png", drawMarks(grayImg, corners_sp));
    cv::imwrite("../img/process/newModes.png", drawModes(grayImg, corners_sp, corners_modes));
#endif
#ifdef SHOW_PROC_IMG
    showImg(drawMarks(grayImg, corners_sp), "newCorners");
    showImg(drawModes(grayImg, corners_sp, corners_modes), "newModes");
    cv::waitKey(0);
#endif

#ifdef TIMING_PROC
    timer.re_start();
#endif
    // generate chess board
    std::pair<bool, CBCorners> cbcs = genChessBoard(computeEach);
#ifdef TIMING_PROC
    LOG_INFO(timer.last_elapsed("genChessBoard"));
#endif

#ifdef WRITE_PROC_IMG
    if (cbcs.first) {
      cv::imwrite("../img/process/chessboard.png", drawChessBoard(grayImg, cbcs.second));
    }
#endif
#ifdef SHOW_PROC_IMG
    if (cbcs.first) {
      showImg(drawChessBoard(grayImg, cbcs.second), "chessboard");
      cv::waitKey(0);
    }
#endif
    destoryHistory();
    return cbcs;
  }

  std::vector<CBCorners>
  Detector::solveMutiCB(cv::Mat gImg) {
    // assign
    this->grayImg = gImg;
#ifdef TIMING_PROC
    ns_timer::Timer timer;
    timer.re_start();
#endif
    // compute likehood
    compute_likehood();
#ifdef TIMING_PROC
    LOG_INFO(timer.last_elapsed("compute_likehood"));
#endif

#ifdef WRITE_PROC_IMG
    cv::imwrite("../img/process_muti/likehood.png", cvt_32FC1_8UC1(likehood));
#endif
#ifdef SHOW_PROC_IMG
    showImg(cvt_32FC1_8UC1(likehood), "likehood");
    cv::waitKey(0);
#endif

#ifdef TIMING_PROC
    timer.re_start();
#endif
    // find possible corners
    findCorners();
#ifdef TIMING_PROC
    LOG_INFO(timer.last_elapsed("findCorners"));
#endif

#ifdef WRITE_PROC_IMG
    cv::imwrite("../img/process_muti/findCorners.png", drawMarks(grayImg, corners));
#endif
#ifdef SHOW_PROC_IMG
    showImg(drawMarks(grayImg, corners), "findCorners");
    cv::waitKey(0);
#endif

#ifdef TIMING_PROC
    timer.re_start();
#endif
    // verify the corners
    verifyCorners();
#ifdef TIMING_PROC
    LOG_INFO(timer.last_elapsed("verifyCorners"));
#endif

#ifdef WRITE_PROC_IMG
    cv::imwrite("../img/process_muti/oldCorners.png", drawMarks(grayImg, corners));
    cv::imwrite("../img/process_muti/oldModes.png", drawModes(grayImg, corners, corners_modes));
#endif
#ifdef SHOW_PROC_IMG
    showImg(drawMarks(grayImg, corners), "oldCorners");
    showImg(drawModes(grayImg, corners, corners_modes), "oldModes");
    cv::waitKey(0);
#endif

#ifdef TIMING_PROC
    timer.re_start();
#endif
    // refine the corners
    refineCorners();
#ifdef TIMING_PROC
    LOG_INFO(timer.last_elapsed("refineCorners"));
#endif

#ifdef WRITE_PROC_IMG
    cv::imwrite("../img/process_muti/newCorners.png", drawMarks(grayImg, corners_sp));
    cv::imwrite("../img/process_muti/newModes.png", drawModes(grayImg, corners_sp, corners_modes));
#endif
#ifdef SHOW_PROC_IMG
    showImg(drawMarks(grayImg, corners_sp), "newCorners");
    showImg(drawModes(grayImg, corners_sp, corners_modes), "newModes");
    cv::waitKey(0);
#endif

#ifdef TIMING_PROC
    timer.re_start();
#endif
    // generate chess boards
    auto cbs = genChessBoard();
#ifdef TIMING_PROC
    LOG_INFO(timer.last_elapsed("genChessBoard"));
#endif

#ifdef WRITE_PROC_IMG
    cv::imwrite("../img/process_muti/chessboard4.png", drawChessBoard(grayImg, cbs));
#endif
#ifdef SHOW_PROC_IMG
    showImg(drawChessBoard(grayImg, cbs), "chessboard");
    cv::waitKey(0);
#endif
    destoryHistory();

    return cbs;
  }

  void Detector::compute_likehood() {
    // two type prototypes [two directions]
    auto proto1 = ns_st10::ProtoType(PROTO_HWS, 0.0, M_PI_2);
    auto proto2 = ns_st10::ProtoType(PROTO_HWS, M_PI_4, M_PI_4 * 3);
    cv::Mat s_type1, s_type2;
    {
      // for the first prototype
      cv::Mat fA, fB, fC, fD;
      cv::filter2D(grayImg, fA, CV_32FC1, proto1.A);
      cv::filter2D(grayImg, fB, CV_32FC1, proto1.B);
      cv::filter2D(grayImg, fC, CV_32FC1, proto1.C);
      cv::filter2D(grayImg, fD, CV_32FC1, proto1.D);
      cv::Mat mu = 0.25f * (fA + fB + fC + fD);
      cv::Mat minAB = cv::min(fA, fB);
      cv::Mat minCD = cv::min(fC, fD);
      cv::Mat s1 = cv::min(cv::Mat(mu - minAB), cv::Mat(minCD - mu));
      cv::Mat s2 = cv::min(cv::Mat(minAB - mu), cv::Mat(mu - minCD));
      s_type1 = cv::max(s1, s2);
    }
    {
      // for the second prototype
      cv::Mat fA, fB, fC, fD;
      cv::filter2D(grayImg, fA, CV_32FC1, proto2.A);
      cv::filter2D(grayImg, fB, CV_32FC1, proto2.B);
      cv::filter2D(grayImg, fC, CV_32FC1, proto2.C);
      cv::filter2D(grayImg, fD, CV_32FC1, proto2.D);
      cv::Mat mu = 0.25f * (fA + fB + fC + fD);
      cv::Mat minAB = cv::min(fA, fB);
      cv::Mat minCD = cv::min(fC, fD);
      cv::Mat s1 = cv::min(cv::Mat(mu - minAB), cv::Mat(minCD - mu));
      cv::Mat s2 = cv::min(cv::Mat(minAB - mu), cv::Mat(mu - minCD));
      s_type2 = cv::max(s1, s2);
    }
    // get the max
    likehood = cv::max(s_type1, s_type2);
    // make the negative values become zero
    likehood = cv::max(0.0f, likehood);
  }

  void Detector::findCorners() {
    // find the max value
    double maxVal;
    cv::minMaxIdx(likehood, nullptr, &maxVal);
    // run the non maximum suppression algorithm
    auto corners_t = nms2d(likehood, NMS_HWS);
    for (const auto &pt : corners_t) {
      // condition
      if (likehood.at<float>(pt) > FIND_CORNERS_THD * maxVal &&
          pt.x >= HIST_HWS &&
          pt.y >= HIST_HWS &&
          pt.x < likehood.cols - HIST_HWS &&
          pt.y < likehood.rows - HIST_HWS) {
        // save the corner
        corners.push_back(pt);
      }
    }
  }

  void Detector::verifyCorners() {
    // rows and colums
    ushort rows = grayImg.rows, cols = grayImg.cols;
    // using sobel filter to filte the image to get gradX and gradY image
    cv::Sobel(grayImg, gradX, CV_32FC1, 1, 0);
    cv::Sobel(grayImg, gradY, CV_32FC1, 0, 1);
    cv::Mat grad, angle;
    // compute the total grad image and angle image
    cv::cartToPolar(gradX, gradY, grad, angle);
    const float binSize = M_PI / 32;
    std::vector<cv::Point> corners_new;
    std::vector<float> scores_new;
    std::vector<std::pair<float, float>> modes_new;

    for (const auto &pt : corners) {
      // for each corner
      int x = pt.x, y = pt.y;
      // the bin
      std::vector<float> bins(32, 0.0f);
      for (int i = y - HIST_HWS; i != y + HIST_HWS + 1; ++i) {
        auto gradPtr = grad.ptr<float>(i);
        auto anglePtr = angle.ptr<float>(i);
        for (int j = x - HIST_HWS; j != x + HIST_HWS + 1; ++j) {
          float g = gradPtr[j], a = anglePtr[j];
          if (a >= M_PI) {
            a -= M_PI;
          }
          // map into the bin
          bins[int(a / binSize)] += g;
        }
      }
      // gauss filter
      gaussFilter(bins);
      // mean shift to find two main modes
      auto modes = meanShift(bins);
      int m1 = modes.first, m2 = modes.second;
      float m1_f = std::min(m1, m2) / 31.0f * M_PI, m2_f = std::max(m1, m2) / 31.0f * M_PI;
      float v1 = bins[m1], v2 = bins[m2];
      int dis = std::abs(m1 - m2);
      if (dis > 16) {
        dis = 32 - dis;
      }
      // condition
      if (dis > 10 && std::abs(v1 - v2) < std::max(v1, v2) * 0.5f) {
        // construct the prototype
        auto proto = ProtoType(HIST_HWS, m1_f, m2_f);
        {
          cv::Mat gx, gy;
          cv::Sobel(proto.A, gx, CV_32FC1, 1, 0);
          cv::Sobel(proto.A, gy, CV_32FC1, 0, 1);
          // proto.A is the grad image, gx is the angle image
          cv::cartToPolar(gx, gy, proto.A, gx);

          cv::Sobel(proto.B, gx, CV_32FC1, 1, 0);
          cv::Sobel(proto.B, gy, CV_32FC1, 0, 1);
          cv::cartToPolar(gx, gy, proto.B, gx);

          cv::Sobel(proto.C, gx, CV_32FC1, 1, 0);
          cv::Sobel(proto.C, gy, CV_32FC1, 0, 1);
          cv::cartToPolar(gx, gy, proto.C, gx);

          cv::Sobel(proto.D, gx, CV_32FC1, 1, 0);
          cv::Sobel(proto.D, gy, CV_32FC1, 0, 1);
          cv::cartToPolar(gx, gy, proto.D, gx);
        }
        // the sub grad image for current corner
        cv::Mat subGrad = grad(cv::Range(y - HIST_HWS, y + HIST_HWS + 1), cv::Range(x - HIST_HWS, x + HIST_HWS + 1));
        // the sub expect grad image for current corner
        cv::Mat subExpGrad = cv::max(cv::Mat(cv::max(proto.A, proto.B)), cv::Mat(cv::max(proto.C, proto.D)));
        // the grad score
        float score_grad = subGrad.dot(subExpGrad) / (cv::norm(subGrad) * cv::norm(subExpGrad));
        // the likehood score
        float score_likehood = likehood.at<float>(pt);
        corners_new.push_back(pt);
        // the total score
        scores_new.push_back(score_likehood * score_grad);
        modes_new.push_back({m1_f, m2_f});
      }
      {
        // draw
        // std::fstream file = std::fstream("../pyDrawer/hist.csv", std::ios::out);
        // for (int i = 0; i != bins.size() - 1; ++i) {
        //   file << bins[i] << ',';
        // }
        // file << bins[bins.size() - 1];
        // file.close();
        // showImg(ns_st10::drawMarks(grayImg, std::vector<cv::Point>{pt}));
        // cv::waitKey(0);
        // LOG_VAR(modes.first, modes.second, dis, v1, v2);
        // int re = system("/bin/python3 /home/csl/CppWorks/artwork/slam-tricks/st10-chessCorner/pyDrawer/drawer.py");
      }
    }
    // find the max score
    float score_max = *std::max_element(scores_new.cbegin(), scores_new.cend());
    corners.clear();
    scores.clear();
    for (int i = 0; i != corners_new.size(); ++i) {
      // condition
      if (scores_new[i] > VERIFY_CORNERS_THD * score_max) {
        corners.push_back(corners_new[i]);
        scores.push_back(scores_new[i]);
        corners_modes.push_back(modes_new[i]);
      }
    }
  }

  void Detector::refineCorners() {
    for (int i = 0; i != corners.size(); ++i) {
      {
        // corner position
        auto &pt = corners[i];
        Eigen::Vector2f c(pt.x, pt.y);
        Eigen::Matrix2f H = Eigen::Matrix2f::Zero();
        Eigen::Vector2f g = Eigen::Vector2f::Zero();

        // gauss newton
        for (int i = pt.y - REFINE_HWS; i != pt.y + REFINE_HWS + 1; ++i) {
          auto gradXPtr = gradX.ptr<float>(i);
          auto gradYPtr = gradY.ptr<float>(i);
          for (int j = pt.x - REFINE_HWS; j != pt.x + REFINE_HWS + 1; ++j) {
            float gx = gradXPtr[j], gy = gradYPtr[j];
            Eigen::Vector2f Jacobian(gx, gy);
            float error = gx * (pt.x - j) + gy * (pt.y - i);
            H += Jacobian * Jacobian.transpose();
            g -= Jacobian * error;
          }
        }
        c += H.ldlt().solve(g);

        corners_sp.push_back(cv::Point2f(c(0), c(1)));
      }

      // angle
      for (int k = 0; k != 2; ++k) {

        auto &modes = corners_modes[i];
        const auto &pt = corners_sp[i];
        auto &alpha1 = modes.first;
        auto &alpha2 = modes.second;
        Eigen::Vector2f vec1(std::cos(alpha1), std::sin(alpha1));
        Eigen::Vector2f vec2(std::cos(alpha2), std::sin(alpha2));
        std::vector<cv::Point2f> grad_alpha1, grad_alpha2;

        // find corners
        for (int i = int(pt.y) - REFINE_HWS; i != int(pt.y) + REFINE_HWS + 1; ++i) {
          auto gradXPtr = gradX.ptr<float>(i);
          auto gradYPtr = gradY.ptr<float>(i);
          for (int j = int(pt.x) - REFINE_HWS; j != int(pt.x) + REFINE_HWS + 1; ++j) {
            float gx = gradXPtr[j], gy = gradYPtr[j];
            Eigen::Vector2f grad(gx, gy);
            grad.normalize();
            if (std::abs(vec1.dot(grad)) < 0.25f) {
              grad_alpha1.push_back(cv::Point2f(gx, gy));
            }
            if (std::abs(vec2.dot(grad)) < 0.25f) {
              grad_alpha2.push_back(cv::Point2f(gx, gy));
            }
          }
        }

        {
          // using SVD method to solve problem
          Eigen::MatrixXf A(grad_alpha1.size(), 2);
          for (int i = 0; i != grad_alpha1.size(); ++i) {
            A(i, 0) = grad_alpha1[i].x;
            A(i, 1) = grad_alpha1[i].y;
          }
          Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullV);
          Eigen::Matrix2f vMatrix = svd.matrixV();
          float vx = vMatrix.col(vMatrix.cols() - 1)(0);
          float vy = vMatrix.col(vMatrix.cols() - 1)(1);
          alpha1 = std::atan2(vy, vx);
        }
        {
          // using SVD method to solve problem
          Eigen::MatrixXf A(grad_alpha2.size(), 2);
          for (int i = 0; i != grad_alpha2.size(); ++i) {
            A(i, 0) = grad_alpha2[i].x;
            A(i, 1) = grad_alpha2[i].y;
          }
          Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullV);
          Eigen::Matrix2f vMatrix = svd.matrixV();
          float vx = vMatrix.col(vMatrix.cols() - 1)(0);
          float vy = vMatrix.col(vMatrix.cols() - 1)(1);
          alpha2 = std::atan2(vy, vx);
        }
      }
    }

    // build kdtree to make search faster
    kdtree = std::make_shared<pcl::KdTreeFLANN<pcl::PointXY>>();
    pcl::PointCloud<pcl::PointXY>::Ptr cloud(new pcl::PointCloud<pcl::PointXY>());
    cloud->points.resize(corners_sp.size());
    for (int i = 0; i != corners_sp.size(); ++i) {
      cloud->points[i] = pcl::PointXY(corners_sp[i].x, corners_sp[i].y);
    }
    kdtree->setInputCloud(cloud);
  }

  std::pair<bool, CBCorners>
  Detector::genChessBoard(bool computeEach) {
    std::size_t size = corners_sp.size();
    std::vector<std::pair<float, ChessBoard>> res;
    for (int i = 0; i != size; ++i) {
      // try generate a chess board started from current corner
      auto [isVaild, engry, board] = genChessBoardOnce(i);
      if (isVaild) {
        res.push_back({engry, board});
        if (!computeEach) {
          break;
        }
      }
    }
    // if no valid chess board found
    if (res.empty()) {
      return {false, CBCorners()};
    }
    // return the chess board whose energy is the minimum
    auto iter = std::min_element(res.cbegin(), res.cend(), [](const auto &p1, const auto &p2) {
      return p1.first < p2.first;
    });
    return {true, iter->second.toCBCorners(corners_sp)};
  }

  std::vector<CBCorners>
  Detector::genChessBoard() {
    std::size_t size = corners_sp.size();
    std::vector<std::pair<float, ChessBoard>> res;
    // find all chess boards
    for (int i = 0; i != size; ++i) {
      auto [isVaild, engry, board] = genChessBoardOnce(i);
      if (isVaild) {
        res.push_back({engry, board});
      }
    }

    if (res.empty()) {
      return std::vector<CBCorners>();
    } else if (res.size() == 1) {
      return {res.front().second.toCBCorners(corners_sp)};
    }

    // sort the chess board according to their energy
    std::sort(res.begin(), res.end(), [](const auto &p1, const auto &p2) { return p1.first < p2.first; });

    // filte the same chess board
    auto iter = std::unique(res.begin(), res.end(), [](const auto &p1, const auto &p2) { return std::abs(p1.first - p2.first) < 1E-5; });
    res.erase(iter, res.end());

    std::vector<std::pair<decltype(res)::iterator, std::set<std::size_t>>> goodCBs;

    // if two chess boards are intersected with each other, then choose the one whoes energy is minimum
    for (auto iter = res.begin(); iter != res.end(); ++iter) {
      const auto &cb = iter->second;
      const auto &energy = iter->first;
      int j = 0;
      for (; j != goodCBs.size(); ++j) {
        bool intersect = false;
        for (int k = 0; k != cb.rows(); ++k) {
          for (int l = 0; l != cb.cols(); ++l) {
            if (goodCBs[j].second.find(cb[k][l]) == goodCBs[j].second.cend()) {
              continue;
            } else {
              intersect = true;
              break;
            }
          }
          if (intersect) {
            break;
          }
        }
        if (intersect) {
          cb.addToSet(goodCBs[j].second);
          if (energy < goodCBs[j].first->first) {
            goodCBs[j].first = iter;
          }
          break;
        }
      }
      if (j == goodCBs.size()) {
        goodCBs.push_back({iter, cb.toSet()});
      }
    }
    // turn the chess boards into cbcorners
    std::vector<CBCorners> cbcs;
    for (const auto &[iter, s] : goodCBs) {
      cbcs.push_back(iter->second.toCBCorners(corners_sp));
    }

    return cbcs;
  }

  std::tuple<bool, float, ChessBoard>
  Detector::genChessBoardOnce(std::size_t idx) {
    // initialize a chess board
    auto res = initChessBoard(idx);
    if (!res.first) {
      return {false, 0.0f, ChessBoard()};
    }
    // test
    ChessBoard &cb = res.second;
    float lastEnergy = res.first;
    while (true) {
      // try grow the chess board in four directions
      auto cb_top = growChessBoard(cb, CBGrowDir::TOP);
      auto cb_lower = growChessBoard(cb, CBGrowDir::LOWER);
      auto cb_left = growChessBoard(cb, CBGrowDir::LEFT);
      auto cb_right = growChessBoard(cb, CBGrowDir::RIGHT);
      // choose the chess board whose energy is minimum
      std::vector<std::pair<float, ns_st10::ChessBoard>> cbs{cb_top, cb_lower, cb_left, cb_right};
      auto iter = std::min_element(cbs.cbegin(), cbs.cend(), [](const auto &p1, const auto &p2) {
        return p1.first < p2.first;
      });
      // display
#ifdef SHOW_GROW_CB
      LOG_VAR(iter->first);
      showImg(drawChessBoard(grayImg, iter->second, corners_sp), "try once [best]");
      cv::waitKey(0);
#endif
      // condition
      if (iter->first < lastEnergy) {
        // grow success
        lastEnergy = iter->first;
        cb = iter->second;
      } else {
        // grow faild
        break;
      }
    }
    return {true, lastEnergy, res.second};
  }

  std::pair<bool, ChessBoard>
  Detector::initChessBoard(std::size_t idx) {
    const cv::Point2f &cp(corners_sp[idx]);
    const auto &mode = corners_modes[idx];
    // the first mode
    Eigen::Vector2f dir1(std::cos(mode.first), std::sin(mode.first));
    // the second mode
    Eigen::Vector2f dir2(std::cos(mode.second), std::sin(mode.second));
    float dist[8];
    // find the other eight corners
    auto lower = closestCornerInDir(idx, dir1, dist[0]);
    if (!lower.first) {
      return {false, ChessBoard()};
    }
    auto top = closestCornerInDir(idx, -dir1, dist[1]);
    if (!top.first) {
      return {false, ChessBoard()};
    }
    auto right = closestCornerInDir(idx, dir2, dist[2]);
    if (!right.first) {
      return {false, ChessBoard()};
    }
    auto left = closestCornerInDir(idx, -dir2, dist[3]);
    if (!left.first) {
      return {false, ChessBoard()};
    }
    auto leftTop = closestCornerInDir(left.second, -dir1, dist[4]);
    if (!leftTop.first) {
      return {false, ChessBoard()};
    }
    auto rightTop = closestCornerInDir(right.second, -dir1, dist[5]);
    if (!rightTop.first) {
      return {false, ChessBoard()};
    }
    auto leftLower = closestCornerInDir(left.second, dir1, dist[6]);
    if (!leftLower.first) {
      return {false, ChessBoard()};
    }
    auto rightLower = closestCornerInDir(right.second, dir1, dist[7]);
    if (!rightLower.first) {
      return {false, ChessBoard()};
    }
    // construct the chess board [3x3]
    ChessBoard board(3, std::deque<size_t>(3));
    board[0][0] = leftTop.second, board[0][1] = top.second, board[0][2] = rightTop.second;
    board[1][0] = left.second, board[1][1] = idx, board[1][2] = right.second;
    board[2][0] = leftLower.second, board[2][1] = lower.second, board[2][2] = rightLower.second;
    // verify
    float mean = 0.0f, sigma = 0.0f;
    {
      for (int i = 0; i != 8; ++i) {
        mean += dist[i];
      }
      mean /= 9.0f;
      for (int i = 0; i != 8; ++i) {
        sigma += (dist[i] - mean) * (dist[i] - mean);
      }
      sigma = std::sqrt(sigma / 9.0f);
    }
    float s = sigma / mean;

    if (s > 0.3f || chessBoardEnergy(board) > 0.0f) {
      return {false, ChessBoard()};
    } else {
      return {true, board};
    }
  }

  std::pair<bool, std::size_t>
  Detector::closestCornerInDir(std::size_t cornerIdx, const Eigen::Vector2f &dir, float &dist) {
    std::size_t size = corners_sp.size();
    // the score
    std::vector<float> s(size, 0.0f);
    // the corner
    const cv::Point2f &cp(corners_sp[cornerIdx]);
    std::size_t validCount = 0;
    for (int i = 0; i != size; ++i) {
      if (i == cornerIdx) {
        s[i] = MAXFLOAT;
        continue;
      }
      Eigen::Vector2f vec(corners_sp[i].x - cp.x, corners_sp[i].y - cp.y);
      float vecNorm = vec.norm();
      // dis = cos(theta) * norm(vec)
      float dis = vec.dot(dir);
      float cosTheta = dis / vecNorm;
      // condition [30 degree]
      if (cosTheta < 0.86) {
        s[i] = MAXFLOAT;
      } else {
        float disEdge = std::sqrt(vecNorm * vecNorm - dis * dis);
        // score
        s[i] = dis + 5.0f * disEdge;
        ++validCount;
      }
    }
    if (validCount == 0) {
      return {false, 0};
    }
    // retirn the corner whose score is minimum
    auto iter = std::min_element(s.cbegin(), s.cend());
    dist = *iter;
    return {true, iter - s.cbegin()};
  }

  float Detector::chessBoardEnergy(const ChessBoard &cb) {
    // find the maximum energy of the triple corners
    float lastEnergy = 0.0f;
    std::size_t rows = cb.rows(), cols = cb.cols();
    // horizontal
    for (int i = 0; i != rows; ++i) {
      for (int j = 1; j != cols - 1; ++j) {
        const cv::Point2f &p1 = corners_sp[cb[i][j - 1]];
        const cv::Point2f &p2 = corners_sp[cb[i][j]];
        const cv::Point2f &p3 = corners_sp[cb[i][j + 1]];
        Eigen::Vector2f ci(p1.x, p1.y), cj(p2.x, p2.y), ck(p3.x, p3.y);
        float energy = (ci + ck - 2.0f * cj).norm() / (ci - ck).norm();
        if (energy > lastEnergy) {
          lastEnergy = energy;
        }
      }
    }
    // vertical
    for (int i = 1; i != rows - 1; ++i) {
      for (int j = 0; j != cols; ++j) {
        const cv::Point2f &p1 = corners_sp[cb[i - 1][j]];
        const cv::Point2f &p2 = corners_sp[cb[i][j]];
        const cv::Point2f &p3 = corners_sp[cb[i + 1][j]];
        Eigen::Vector2f ci(p1.x, p1.y), cj(p2.x, p2.y), ck(p3.x, p3.y);
        float energy = (ci + ck - 2.0f * cj).norm() / (ci - ck).norm();
        if (energy > lastEnergy) {
          lastEnergy = energy;
        }
      }
    }
    // compute the energy
    float energy = (lastEnergy - 1.0f) * rows * cols;
    return energy;
  }

  std::pair<float, ChessBoard>
  Detector::growChessBoard(const ChessBoard &cb, CBGrowDir dir) {
    std::vector<int> pointIdxKNNSearch(1);
    std::vector<float> pointKNNSquaredDistance(1);
    ChessBoard cb_pred = cb;
    switch (dir) {
    // four cases
    case CBGrowDir::TOP: {
      cb_pred.push_front(std::deque<std::size_t>(cb.cols()));
      std::size_t rows = cb_pred.rows(), cols = cb_pred.cols();
      for (int i = 0; i != cols; ++i) {
        const cv::Point2f &p1 = corners_sp[cb_pred[2][i]];
        const cv::Point2f &p2 = corners_sp[cb_pred[1][i]];
        // predict corner
        cv::Point2f predPt = predictCorner(p1, p2);
        // find the closest corner
        kdtree->nearestKSearch(pcl::PointXY(predPt.x, predPt.y), 1, pointIdxKNNSearch, pointKNNSquaredDistance);
        cb_pred[0][i] = pointIdxKNNSearch[0];
      }
    } break;
    case CBGrowDir::LOWER: {
      cb_pred.push_back(std::deque<std::size_t>(cb.cols()));
      std::size_t rows = cb_pred.rows(), cols = cb_pred.cols();
      for (int i = 0; i != cols; ++i) {
        const cv::Point2f &p1 = corners_sp[cb_pred[rows - 3][i]];
        const cv::Point2f &p2 = corners_sp[cb_pred[rows - 2][i]];
        cv::Point2f predPt = predictCorner(p1, p2);
        kdtree->nearestKSearch(pcl::PointXY(predPt.x, predPt.y), 1, pointIdxKNNSearch, pointKNNSquaredDistance);
        cb_pred[rows - 1][i] = pointIdxKNNSearch[0];
      }
    } break;
    case CBGrowDir::LEFT: {
      for (int i = 0; i != cb.size(); ++i) {
        cb_pred[i].push_front(0);
      }
      std::size_t rows = cb_pred.rows(), cols = cb_pred.cols();
      for (int i = 0; i != rows; ++i) {
        const cv::Point2f &p1 = corners_sp[cb_pred[i][2]];
        const cv::Point2f &p2 = corners_sp[cb_pred[i][1]];
        cv::Point2f predPt = predictCorner(p1, p2);
        kdtree->nearestKSearch(pcl::PointXY(predPt.x, predPt.y), 1, pointIdxKNNSearch, pointKNNSquaredDistance);
        cb_pred[i][0] = pointIdxKNNSearch[0];
      }
    } break;
    case CBGrowDir::RIGHT: {
      for (int i = 0; i != cb.rows(); ++i) {
        cb_pred[i].push_back(0);
      }
      std::size_t rows = cb_pred.rows(), cols = cb_pred.cols();
      for (int i = 0; i != rows; ++i) {
        const cv::Point2f &p1 = corners_sp[cb_pred[i][cols - 3]];
        const cv::Point2f &p2 = corners_sp[cb_pred[i][cols - 2]];
        cv::Point2f predPt = predictCorner(p1, p2);
        kdtree->nearestKSearch(pcl::PointXY(predPt.x, predPt.y), 1, pointIdxKNNSearch, pointKNNSquaredDistance);
        cb_pred[i][cols - 1] = pointIdxKNNSearch[0];
      }
    } break;

    default:
      break;
    }
    return {chessBoardEnergy(cb_pred), cb_pred};
  }

  cv::Point2f Detector::predictCorner(const cv::Point2f &p1, const cv::Point2f &p2) {
    // 0.75 is a experiential factor
    float predX = p2.x + (p2.x - p1.x) * 0.75f;
    float predY = p2.y + (p2.y - p1.y) * 0.75f;
    return cv::Point2f(predX, predY);
  }

  void Detector::destoryHistory() {
    this->corners.clear();
    this->corners_sp.clear();
    this->scores.clear();
    this->corners_modes.clear();
  }
} // namespace ns_st10
