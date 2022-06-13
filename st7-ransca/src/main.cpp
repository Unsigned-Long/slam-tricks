#include "parabola.hpp"

int main(int argc, char const *argv[]) {
  ns_st7::parabola para{1, 2, 3};
  auto [good, bad] = ns_st7::genData(para);
  good.write("../data/good.csv", std::ios::out);
  bad.write("../data/bad.csv", std::ios::out);
  LOG_VAR(ns_st7::Solver::leastSquare(good));
  LOG_VAR(ns_st7::Solver::leastSquare(bad));
  LOG_VAR(ns_st7::Solver::gaussNewton(good, 10));
  LOG_VAR(ns_st7::Solver::gaussNewton(bad, 10));
  LOG_VAR(ns_st7::Solver::ransac(good, 10, 0.3f));
  LOG_VAR(ns_st7::Solver::ransac(bad, 10, 0.3f));
  return 0;
}
