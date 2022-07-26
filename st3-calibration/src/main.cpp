#include "calib.h"

int main(int argc, char const *argv[]) {
  ns_st3::CalibSolver solver("../calib", 2.8E-2);
  solver.solve();
  return 0;
}
