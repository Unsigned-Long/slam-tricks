#include "artwork/csv/csv.h"
#include "icp.hpp"

constexpr ushort PTS_COUNT = 10;

int main(int argc, char const *argv[]) {
  auto writer1 = ns_csv::CSVWriter::create("../log/nobinding/pc1.csv");
  auto writer2 = ns_csv::CSVWriter::create("../log/nobinding/pc2.csv");
  auto writer3 = ns_csv::CSVWriter::create("../log/nobinding/pc1_prime_4.csv");
  // pre
  Sophus::SE2f T21_truth(M_PI_4, Eigen::Vector2f(2.0f, 2.0f));
  auto [pc1, pc2] = ns_st6::generatePts(T21_truth, PTS_COUNT);
  for (const auto &elem : pc1) {
    writer1->writeLine(',', elem(0), elem(1));
  }
  for (const auto &elem : pc2) {
    writer2->writeLine(',', elem(0), elem(1));
  }
  // solve
  auto T21_est = ns_st6::icp_no_binding(pc1, pc2, 4);
  // transfrom
  for (int i = 0; i != PTS_COUNT; ++i) {
    auto p1 = pc1.at(i);
    auto p1_prime = T21_est * p1;
    writer3->writeLine(',', p1_prime[0], p1_prime[1]);
  }
  return 0;
}
