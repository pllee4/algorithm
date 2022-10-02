/*
 * bresenham_test.cpp
 *
 * Created on: Sep 29, 2022 22:18
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#include "algorithm/robotics/bresenham/bresenham.hpp"

#include "gtest/gtest.h"

using namespace pllee4;
using namespace graph;

TEST(Bresenham, CheckLineFromNegativeTowardsRightDownwards) {
  Coordinate start{-5, 2};
  Coordinate end{2, 1};
  LineGenerator line(start, end);
  auto points = line.GetPoints();
  std::vector<Coordinate> expected = {{-5, 2}, {-4, 2}, {-3, 2}, {-2, 2},
                                      {-1, 1}, {0, 1},  {1, 1},  {2, 1}};
  EXPECT_TRUE(std::equal(std::begin(points), std::end(points),
                         std::begin(expected), std::end(expected)));
}

TEST(Bresenham, CheckLineFromPositiveTowardsRightUpwards) {
  Coordinate start{3, 2};
  Coordinate end{15, 5};
  LineGenerator line(start, end);
  auto points = line.GetPoints();
  std::vector<Coordinate> expected = {
      {3, 2},  {4, 2},  {5, 3},  {6, 3},  {7, 3},  {8, 3}, {9, 4},
      {10, 4}, {11, 4}, {12, 4}, {13, 5}, {14, 5}, {15, 5}};
  EXPECT_TRUE(std::equal(std::begin(points), std::end(points),
                         std::begin(expected), std::end(expected)));
}
