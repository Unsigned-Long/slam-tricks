#include "detector.h"

namespace ns_st10 {
  Detector::Detector(ushort protoHWS, ushort nmsHWS, ushort histHWS, ushort refineHWS)
      : PROTO_HWS(protoHWS), NMS_HWS(nmsHWS), HIST_HWS(histHWS), REFINE_HWS(refineHWS) {}

  std::pair<bool, CBCorners>
  Detector::solve(cv::Mat gImg, bool computeEach) {
    this->grayImg = gImg;
    compute_likehood();
#ifdef WRITE_PROC_IMG
    cv::imwrite("../img/process/likehood.png", cvt_32FC1_8UC1(likehood));
#endif
#ifdef SHOW_PROC_IMG
    showImg(cvt_32FC1_8UC1(likehood), "likehood");
    cv::waitKey(0);
#endif

    findCorners();
#ifdef WRITE_PROC_IMG
    cv::imwrite("../img/process/findCorners.png", drawMarks(grayImg, corners));
#endif
#ifdef SHOW_PROC_IMG
    showImg(drawMarks(grayImg, corners), "findCorners");
    cv::waitKey(0);
#endif

    verifyCorners();
#ifdef WRITE_PROC_IMG
    cv::imwrite("../img/process/oldCorners.png", drawMarks(grayImg, corners));
    cv::imwrite("../img/process/oldModes.png", drawModes(grayImg, corners, corners_modes));
#endif
#ifdef SHOW_PROC_IMG
    showImg(drawMarks(grayImg, corners), "oldCorners");
    showImg(drawModes(grayImg, corners, corners_modes), "oldModes");
    cv::waitKey(0);
#endif

    refineCorners();
#ifdef WRITE_PROC_IMG
    cv::imwrite("../img/process/newCorners.png", drawMarks(grayImg, corners_sp));
    cv::imwrite("../img/process/newModes.png", drawModes(grayImg, corners_sp, corners_modes));
#endif
#ifdef SHOW_PROC_IMG
    showImg(drawMarks(grayImg, corners_sp), "newCorners");
    showImg(drawModes(grayImg, corners_sp, corners_modes), "newModes");
    cv::waitKey(0);
#endif

    std::pair<bool, CBCorners> cbcs = genChessBoard(computeEach);
    if (cbcs.first) {
      cbcs.second.adjust();
    }
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
    return cbcs;
  }

  void Detector::compute_likehood() {
    auto proto1 = ns_st10::ProtoType(PROTO_HWS, 0.0, M_PI_2);
    auto proto2 = ns_st10::ProtoType(PROTO_HWS, M_PI_4, M_PI_4 * 3);
    cv::Mat s_type1, s_type2;
    {
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
    likehood = cv::max(s_type1, s_type2);
    likehood = cv::max(0.0f, likehood);
  }

  void Detector::findCorners() {
    double maxVal;
    cv::minMaxIdx(likehood, nullptr, &maxVal);
    auto corners_t = nms2d(likehood, NMS_HWS);
    for (const auto &pt : corners_t) {
      if (likehood.at<float>(pt) > 0.5 * maxVal &&
          pt.x >= HIST_HWS &&
          pt.y >= HIST_HWS &&
          pt.x < likehood.cols - HIST_HWS &&
          pt.y < likehood.rows - HIST_HWS) {
        corners.push_back(pt);
      }
    }
    // display
    LOG_VAR(corners.size());
  }

  void Detector::verifyCorners() {
    ushort rows = grayImg.rows, cols = grayImg.cols;
    cv::Sobel(grayImg, gradX, CV_32FC1, 1, 0);
    cv::Sobel(grayImg, gradY, CV_32FC1, 0, 1);
    cv::Mat grad, angle;
    cv::cartToPolar(gradX, gradY, grad, angle);
    const float binSize = M_PI / 32;
    std::vector<cv::Point> corners_new;
    std::vector<float> scores_new;
    std::vector<std::pair<float, float>> modes_new;

    for (const auto &pt : corners) {
      int x = pt.x, y = pt.y;
      std::vector<float> bins(32, 0.0f);
      for (int i = y - HIST_HWS; i != y + HIST_HWS + 1; ++i) {
        auto gradPtr = grad.ptr<float>(i);
        auto anglePtr = angle.ptr<float>(i);
        for (int j = x - HIST_HWS; j != x + HIST_HWS + 1; ++j) {
          float g = gradPtr[j], a = anglePtr[j];
          if (a >= M_PI) {
            a -= M_PI;
          }
          bins[int(a / binSize)] += g;
        }
      }
      // gauss filter
      gaussFilter(bins);
      // mean shift
      auto modes = meanShift(bins);
      int m1 = modes.first, m2 = modes.second;
      float m1_f = std::min(m1, m2) / 31.0f * M_PI, m2_f = std::max(m1, m2) / 31.0f * M_PI;
      float v1 = bins[m1], v2 = bins[m2];
      int dis = std::abs(m1 - m2);
      if (dis > 16) {
        dis = 32 - dis;
      }
      if (dis > 10 && std::abs(v1 - v2) < std::max(v1, v2) * 0.5f) {
        // compute score
        auto proto = ProtoType(HIST_HWS, m1_f, m2_f);
        {
          cv::Mat gx, gy;
          cv::Sobel(proto.A, gx, CV_32FC1, 1, 0);
          cv::Sobel(proto.A, gy, CV_32FC1, 0, 1);
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
        cv::Mat subGrad = grad(cv::Range(y - HIST_HWS, y + HIST_HWS + 1), cv::Range(x - HIST_HWS, x + HIST_HWS + 1));
        cv::Mat subExpGrad = cv::max(cv::Mat(cv::max(proto.A, proto.B)), cv::Mat(cv::max(proto.C, proto.D)));
        float score_grad = subGrad.dot(subExpGrad) / (cv::norm(subGrad) * cv::norm(subExpGrad));
        float score_likehood = likehood.at<float>(pt);
        // is a good corner
        corners_new.push_back(pt);
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
    float score_max = *std::max_element(scores_new.cbegin(), scores_new.cend());
    corners.clear();
    scores.clear();
    for (int i = 0; i != corners_new.size(); ++i) {
      if (scores_new[i] > 0.4f * score_max) {
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

    // build kdtree
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
    std::vector<std::pair<float, Detector::ChessBoard>> res;
    for (int i = 0; i != size; ++i) {
      auto [isVaild, engry, board] = genChessBoardOnce(i);
      if (isVaild) {
        res.push_back({engry, board});
        if (!computeEach) {
          break;
        }
      }
    }
    if (res.empty()) {
      return {false, CBCorners()};
    }
    auto iter = std::min_element(res.cbegin(), res.cend(), [](const auto &p1, const auto &p2) {
      return p1.first < p2.first;
    });
    const auto &board = iter->second;
    CBCorners cbcs(board.size(), std::vector<cv::Point2f>(board.front().size()));
    for (int i = 0; i != board.size(); ++i) {
      for (int j = 0; j != board[0].size(); ++j) {
        cbcs[i][j] = corners_sp[board[i][j]];
      }
    }
    return {true, cbcs};
  }

  std::tuple<bool, float, Detector::ChessBoard>
  Detector::genChessBoardOnce(std::size_t idx) {
    auto res = initChessBoard(idx);
    if (!res.first) {
      return {false, 0.0f, ChessBoard()};
    }
    // test
    ChessBoard &cb = res.second;
    float lastEnergy = res.first;
    while (true) {
      auto cb_top = growChessBoard(cb, CBGrowDir::TOP);
      auto cb_lower = growChessBoard(cb, CBGrowDir::LOWER);
      auto cb_left = growChessBoard(cb, CBGrowDir::LEFT);
      auto cb_right = growChessBoard(cb, CBGrowDir::RIGHT);
      std::vector<std::pair<float, ns_st10::Detector::ChessBoard>> cbs{cb_top, cb_lower, cb_left, cb_right};
      auto iter = std::min_element(cbs.cbegin(), cbs.cend(), [](const auto &p1, const auto &p2) {
        return p1.first < p2.first;
      });
      // display
#ifdef SHOW_GROW_CB
      LOG_VAR(iter->first);
      showImg(drawChessBoard(grayImg, iter->second, corners_sp), "try once [best]");
      cv::waitKey(0);
#endif
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

  std::pair<bool, Detector::ChessBoard>
  Detector::initChessBoard(std::size_t idx) {
    const cv::Point2f &cp(corners_sp[idx]);
    const auto &mode = corners_modes[idx];
    Eigen::Vector2f dir1(std::cos(mode.first), std::sin(mode.first));
    Eigen::Vector2f dir2(std::cos(mode.second), std::sin(mode.second));
    float dist[8];
    auto lower = closestCornerInDir(idx, dir1, dist[0]);
    if (!lower.first) {
      return {false, Detector::ChessBoard()};
    }
    auto top = closestCornerInDir(idx, -dir1, dist[1]);
    if (!top.first) {
      return {false, Detector::ChessBoard()};
    }
    auto right = closestCornerInDir(idx, dir2, dist[2]);
    if (!right.first) {
      return {false, Detector::ChessBoard()};
    }
    auto left = closestCornerInDir(idx, -dir2, dist[3]);
    if (!left.first) {
      return {false, Detector::ChessBoard()};
    }
    auto leftTop = closestCornerInDir(left.second, -dir1, dist[4]);
    if (!leftTop.first) {
      return {false, Detector::ChessBoard()};
    }
    auto rightTop = closestCornerInDir(right.second, -dir1, dist[5]);
    if (!rightTop.first) {
      return {false, Detector::ChessBoard()};
    }
    auto leftLower = closestCornerInDir(left.second, dir1, dist[6]);
    if (!leftLower.first) {
      return {false, Detector::ChessBoard()};
    }
    auto rightLower = closestCornerInDir(right.second, dir1, dist[7]);
    if (!rightLower.first) {
      return {false, Detector::ChessBoard()};
    }
    Detector::ChessBoard board(3, std::deque<size_t>(3));
    board[0][0] = leftTop.second, board[0][1] = top.second, board[0][2] = rightTop.second;
    board[1][0] = left.second, board[1][1] = idx, board[1][2] = right.second;
    board[2][0] = leftLower.second, board[2][1] = lower.second, board[2][2] = rightLower.second;
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
      return {false, Detector::ChessBoard()};
    } else {
      return {true, board};
    }
  }

  std::pair<bool, std::size_t>
  Detector::closestCornerInDir(std::size_t cornerIdx, const Eigen::Vector2f &dir, float &dist) {
    std::size_t size = corners_sp.size();
    std::vector<float> s(size, 0.0f);
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
      if (cosTheta < 0.86) {
        s[i] = MAXFLOAT;
      } else {
        float disEdge = std::sqrt(vecNorm * vecNorm - dis * dis);
        s[i] = dis + 5.0f * disEdge;
        ++validCount;
      }
    }
    if (validCount == 0) {
      return {false, 0};
    }
    auto iter = std::min_element(s.cbegin(), s.cend());
    dist = *iter;
    return {true, iter - s.cbegin()};
  }

  float Detector::chessBoardEnergy(const Detector::ChessBoard &cb) {
    float lastEnergy = 0.0f;
    std::size_t rows = cb.size(), cols = cb.front().size();
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
    float energy = (lastEnergy - 1.0f) * rows * cols;
    return energy;
  }

  std::pair<float, Detector::ChessBoard>
  Detector::growChessBoard(const Detector::ChessBoard &cb, CBGrowDir dir) {
    std::vector<int> pointIdxKNNSearch(1);
    std::vector<float> pointKNNSquaredDistance(1);
    Detector::ChessBoard cb_pred = cb;
    switch (dir) {
    case CBGrowDir::TOP: {
      cb_pred.push_front(std::deque<std::size_t>(cb.front().size()));
      std::size_t rows = cb_pred.size(), cols = cb_pred.front().size();
      for (int i = 0; i != cols; ++i) {
        const cv::Point2f &p1 = corners_sp[cb_pred[2][i]];
        const cv::Point2f &p2 = corners_sp[cb_pred[1][i]];
        cv::Point2f predPt = predictCorner(p1, p2);
        kdtree->nearestKSearch(pcl::PointXY(predPt.x, predPt.y), 1, pointIdxKNNSearch, pointKNNSquaredDistance);
        cb_pred[0][i] = pointIdxKNNSearch[0];
      }
    } break;
    case CBGrowDir::LOWER: {
      cb_pred.push_back(std::deque<std::size_t>(cb.front().size()));
      std::size_t rows = cb_pred.size(), cols = cb_pred.front().size();
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
      std::size_t rows = cb_pred.size(), cols = cb_pred.front().size();
      for (int i = 0; i != rows; ++i) {
        const cv::Point2f &p1 = corners_sp[cb_pred[i][2]];
        const cv::Point2f &p2 = corners_sp[cb_pred[i][1]];
        cv::Point2f predPt = predictCorner(p1, p2);
        kdtree->nearestKSearch(pcl::PointXY(predPt.x, predPt.y), 1, pointIdxKNNSearch, pointKNNSquaredDistance);
        cb_pred[i][0] = pointIdxKNNSearch[0];
      }
    } break;
    case CBGrowDir::RIGHT: {
      for (int i = 0; i != cb.size(); ++i) {
        cb_pred[i].push_back(0);
      }
      std::size_t rows = cb_pred.size(), cols = cb_pred.front().size();
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
    float predX = p2.x + (p2.x - p1.x) * 0.75f;
    float predY = p2.y + (p2.y - p1.y) * 0.75f;
    return cv::Point2f(predX, predY);
  }
} // namespace ns_st10
