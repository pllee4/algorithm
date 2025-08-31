/*
 * bresenham.cpp
 *
 * Created on: Sep 28, 2022 20:58
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#include "algorithm/robotics/bresenham/bresenham.hpp"

#include <cstdlib>

namespace pllee4::graph {
LineGenerator::LineGenerator(const Coordinate& start, const Coordinate& end) {
  const auto dx = abs(end.x - start.x);
  const auto sx = start.x < end.x ? 1 : -1;
  const auto dy = -abs(end.y - start.y);
  const auto sy = start.y < end.y ? 1 : -1;
  auto error = dx + dy;

  auto y = start.y;
  auto x = start.x;

  points_.reserve(std::max(abs(end.x - start.x), abs(end.y - start.y)));

  while (true) {
    points_.emplace_back(Coordinate{x, y});
    if (start == end) break;

    const auto e2 = 2 * error;
    bool should_break = false;

    if (e2 >= dy) {
      if (x == end.x) {
        should_break = true;
      } else {
        error += dy;
        x += sx;
      }
    }

    if (!should_break && e2 <= dx) {
      if (y == end.y) {
        should_break = true;
      } else {
        error += dx;
        y += sy;
      }
    }

    if (should_break) break;
  }
}

std::vector<Coordinate> LineGenerator::GetPoints() const { return points_; }

}  // namespace pllee4::graph
