#include <chrono>
#include <thread>

#include "tester.hpp"
#include "solver.hpp"

int solver(const std::shared_ptr<backend_interface::Tester> tester, const bool preempt) {
  solution::Solver solver(tester, preempt);
  solver.start();

  std::this_thread::sleep_for(std::chrono::hours(1));

  return 0;
}
