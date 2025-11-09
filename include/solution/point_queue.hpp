#pragma once

#include <vector>

#include "tester.hpp"

namespace solution {
  struct QueuedPoint {
    Point point{};
    bool horizontal_motor_done = false;
    bool vertical_motor_done = false;
  };

  using PointQueue_t = std::vector<QueuedPoint>;
}