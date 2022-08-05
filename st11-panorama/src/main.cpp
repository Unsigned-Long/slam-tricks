#include "help.hpp"
#include "panorama.h"
#include "projective.h"

int main(int argc, char const *argv[]) {
  ns_st11::panorama(ns_st11::filesInDir("../imgs/main"));
  return 0;
}
