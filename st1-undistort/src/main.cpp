#include "artwork/timer/timer.h"
#include "opencv2/opencv.hpp"
#include "undistort_img.hpp"
#include "undistort_pt.hpp"

void handleImage(cv::Mat distortedImg,
                 const CameraInnerParam &innerParam,
                 const CameraDistCoeff &distCoff) {
  ns_timer::Timer timer;
  {
    timer.re_start();
    cv::Mat undistortedImg = ns_st1::undistortImage(distortedImg, innerParam, distCoff, Interpolation::NEAREST_NEIGHBOR);
    std::cout << timer.last_elapsed("undistorted-nearest") << std::endl;

    cv::imshow("distortedImg", distortedImg);
    cv::imshow("undistortedImg", undistortedImg);

    cv::waitKey(0);

    cv::imwrite("../img/undistorted-nearest.png", undistortedImg);
  }

  {
    timer.re_start();
    cv::Mat undistortedImg = ns_st1::undistortImage(distortedImg, innerParam, distCoff, Interpolation::BILINEAR);
    std::cout << timer.last_elapsed("undistorted-bilinear") << std::endl;

    cv::imshow("distortedImg", distortedImg);
    cv::imshow("undistortedImg", undistortedImg);

    cv::waitKey(0);

    cv::imwrite("../img/undistorted-bilinear.png", undistortedImg);
  }

  return;
}

float undistortPointError(cv::Point2f srcPt, cv::Point2f dstPt,
                          const CameraInnerParam &innerParam,
                          const CameraDistCoeff &distCoff) {
  float fx = innerParam.fx, fy = innerParam.fy, cx = innerParam.cx, cy = innerParam.cy;
  float k1 = distCoff.k1, k2 = distCoff.k2, k3 = distCoff.k3;
  float p1 = distCoff.p1, p2 = distCoff.p2;
  float x_est = dstPt.x, y_est = dstPt.y;

  float x = (x_est - cx) / fx;
  float y = (y_est - cy) / fy;
  float r2 = x * x + y * y, r4 = r2 * r2, r6 = r4 * r2;

  // the distortion model
  float x_dist = x * (1 + k1 * r2 + k2 * r4 + k3 * r6) + 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
  float y_dist = y * (1 + k1 * r2 + k2 * r4 + k3 * r6) + 2 * p2 * x * y + p1 * (r2 + 2 * y * y);

  // transfrom to pixel coordinate
  float u_dist = x_dist * fx + cx;
  float v_dist = y_dist * fy + cy;

  float eu = srcPt.x - u_dist;
  float ev = srcPt.y - v_dist;

  return std::sqrt(eu * eu + ev * ev);
}

void handlePoint(cv::Mat distortedImg,
                 const CameraInnerParam &innerParam,
                 const CameraDistCoeff &distCoff) {
  ns_timer::Timer timer;
  cv::Point2f srcPt(distortedImg.cols / 4.0f, distortedImg.rows / 4.0f);
  {
    std::cout << std::fixed << std::setprecision(5);
    std::cout << "------------" << std::endl;
    std::cout << "gauss-newton" << std::endl;
    std::cout << "------------" << std::endl;

    timer.re_start();
    auto undistortedPt = ns_st1::undistortPoint(srcPt, innerParam, distCoff);
    std::cout << timer.last_elapsed("undistort point cost") << std::endl;

    std::cout << "srcPt(distorted): " << srcPt << std::endl;
    std::cout << "dstPt(undistorted): " << undistortedPt << std::endl;
    std::cout << std::fixed << std::setprecision(15);
    std::cout << "error: " << undistortPointError(srcPt, undistortedPt, innerParam, distCoff) << std::endl;
  }
  {
    std::cout << std::fixed << std::setprecision(5);
    std::cout << "------" << std::endl;
    std::cout << "OpenCV" << std::endl;
    std::cout << "------" << std::endl;

    cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    K.at<float>(0, 0) = innerParam.fx;
    K.at<float>(1, 1) = innerParam.fy;
    K.at<float>(0, 2) = innerParam.cx;
    K.at<float>(1, 2) = innerParam.cy;
    cv::Mat DistCoef(5, 1, CV_32F);
    DistCoef.at<float>(0) = distCoff.k1;
    DistCoef.at<float>(1) = distCoff.k2;
    DistCoef.at<float>(2) = distCoff.p1;
    DistCoef.at<float>(3) = distCoff.p2;
    DistCoef.at<float>(4) = distCoff.k3;

    cv::Mat mat(1, 2, CV_32F);
    mat.at<float>(0, 0) = srcPt.x;
    mat.at<float>(0, 1) = srcPt.y;

    mat = mat.reshape(1);

    timer.re_start();
    cv::undistortPoints(mat, mat, K, DistCoef, cv::Mat(), K);
    std::cout << timer.last_elapsed("undistort point cost") << std::endl;

    mat = mat.reshape(2);
    std::cout << "srcPt(distorted): " << srcPt << std::endl;
    std::cout << "dstPt(undistorted): " << mat.row(0) << std::endl;
    std::cout << std::fixed << std::setprecision(15);
    std::cout << "error: " << undistortPointError(srcPt, cv::Point2f(mat.at<float>(0, 0), mat.at<float>(0, 1)), innerParam, distCoff) << std::endl;
  }
  return;
}

int main(int argc, char const *argv[]) {
  // parameters
  double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;
  double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.76187114e-05;

  // structures for camera parameters
  CameraInnerParam innerParam(fx, fy, cx, cy);
  CameraDistCoeff distCoff(k1, k2, 0.0, p1, p2);

  // read image
  cv::Mat img = cv::imread("../img/distorted.png", cv::IMREAD_GRAYSCALE);
  try {
    // handle
    ::handleImage(img, innerParam, distCoff);
    ::handlePoint(img, innerParam, distCoff);
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
  }

  return 0;
}
