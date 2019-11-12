#ifndef RANDOM_FAILURE_HPP
#define RANDOM_FAILURE_HPP

#include "random_failure.hpp"
#include <iostream>
#include <random>

struct random_gen {
  std::random_device
      rd; // Will be used to obtain a seed for the random number engine
  std::mt19937 gen; // Standard mersenne_twister_engine seeded with rd()
  std::uniform_int_distribution<> dis;
  random_gen() : rd(), gen(rd()), dis(0, 100) {}

  auto next() { return dis(gen); }
};

int random_value() {
  static random_gen rg;
  return rg.next();
}

// return true with probability x%
int maybe_failure(int x) {
  auto f = random_value();
  std::cout << "failure val " << f << std::endl;
  return f < x;
}

#endif // RANDOM_FAILURE_HPP
