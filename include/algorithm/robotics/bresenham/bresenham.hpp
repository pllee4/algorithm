/*
 * bresenham.hpp
 *
 * Created on: Sep 28, 2022 20:51
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#ifndef BRESENHAM_HPP
#define BRESENHAM_HPP

#include <utility>
#include <vector>

#include "algorithm/robotics/shared_type/data_type.hpp"

namespace pllee4::graph {
class LineGenerator {
 public:
  LineGenerator(const Coordinate& start, const Coordinate& end);
  std::vector<Coordinate> GetPoints() const;

 private:
  std::vector<Coordinate> points_;
};
}  // namespace pllee4::graph
#endif /* BRESENHAM_HPP */
