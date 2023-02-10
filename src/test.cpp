#include <sys/types.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <numeric>
#include <stack>

#include "gsl/gsl_fit.h"

struct Timer {
  Timer() {
    m_start = std::chrono::high_resolution_clock().now();
  }
  ~Timer() {
    m_end = std::chrono::high_resolution_clock().now();
    std::cout << "It took = " << std::chrono::duration_cast<std::chrono::microseconds>(m_end - m_start).count() << " micros" << std::endl;
  }

 private:
  std::chrono::system_clock::time_point m_start;
  std::chrono::system_clock::time_point m_end;
};

int main() {
  const size_t len = 1e6;
  std::array<double, len> x;
  {
    Timer timer;
    for (size_t i = 0; i < x.size(); ++i) {
      x.at(i) = i;
    }
  }
  std::cout << std::accumulate(x.cbegin(), x.cend(), 0) << std::endl;

  std::array<double, len> y;
  {
    Timer timer;
    for (size_t i = 0; i < y.size(); ++i) {
      y.at(i) = std::sin(3.1415 * i / 180.0f);
    }
  }
  std::cout << std::accumulate(y.cbegin(), y.cend(), 0) << std::endl;

  std::array<double, len> w;
  {
    Timer timer;
    for (size_t i = 0; i < w.size(); ++i) {
      w.at(i) = 1.0;
    }
  }
  std::cout << std::accumulate(w.cbegin(), w.cend(), 0) << std::endl;

  double c0, c1, c00, c01, c11, sum;
  {
    Timer timer;
    gsl_fit_wlinear(x.cbegin(), 1, w.cbegin(), 1, y.cbegin(), 1, x.size(), &c0, &c1, &c00, &c01, &c11, &sum);
  }
  std::cout << "Linear: " << c0 << " , " << c1 << std::endl;

  {
    Timer timer;
    gsl_fit_wlinear(x.cbegin(), 1, w.cbegin(), 1, y.cbegin(), 1, x.size(), &c0, &c1, &c00, &c01, &c11, &sum);
  }
  std::cout << "Wlinear: " << c0 << " , " << c1 << std::endl;
  return 0;
}
