#include "calib.h"

int main(int argc, char const *argv[]) {
  ns_st3::CalibSolver solver("../calib", 2.8E-2);
  solver.solve();
  LOG_VAR(solver);
  solver.testUndistortImg("../img/3.jpg");
  return 0;
}
