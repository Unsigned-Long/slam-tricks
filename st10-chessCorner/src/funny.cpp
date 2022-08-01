#include "detector.h"
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

int main(int argc, char const *argv[]) {

  Mat frame;
  //--- INITIALIZE VIDEOCAPTURE
  VideoCapture cap;
  // open the default camera using default API
  // cap.open(0);
  // OR advance usage: select any API backend
  int deviceID = 0;        // 0 = open default camera
  int apiID = cv::CAP_ANY; // 0 = autodetect default API
  // open selected camera using selected API
  cap.open(deviceID, apiID);
  // check if we succeeded
  if (!cap.isOpened()) {
    cerr << "ERROR! Unable to open camera\n";
    return -1;
  }
  //--- GRAB AND WRITE LOOP
  cout << "Start grabbing" << endl
       << "Press any key to terminate" << endl;
  auto solver = ns_st10::Detector(0.2f, 0.15f);

  for (;;) {
    // wait for a new frame from camera and store it into 'frame'
    cap.read(frame);
    // check if we succeeded
    if (frame.empty()) {
      cerr << "ERROR! blank frame grabbed\n";
      break;
    }

    cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
    // auto res = solver.solve(frame, true);
    auto res = solver.solveMutiCB(frame);
    if (!res.empty()) {
      ns_st10::showImg(ns_st10::drawChessBoard(frame, res));
      LOG_PROCESS("success");
      waitKey(5);
    } else {
      LOG_VAR("failed");
    }
  }
  // the camera will be deinitialized automatically in VideoCapture destructor
  return 0;
}
