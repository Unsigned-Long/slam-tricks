#include "pose_simulation.h"
#include "solver.h"

int main(int argc, char const *argv[]) {
  auto data = ns_st4::simulation(30.0f);
  ns_st4::visualization(data);
  return 0;
}
