/*
 * main.cpp
 *
 * Created on: Mar 11, 2022 21:17
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#include <iostream>

#include "dijikstra/dijikstra.hpp"

using namespace pllee4;
using namespace graph;

int main(int argc, char **argv) {
  struct MotionConstraint motion_constraint;
  motion_constraint.dx = {1, 0, -1, 0};
  motion_constraint.dy = {0, 1, 0, -1};
  // motion_constraint.dx = {1, 0, -1, 0, 1, 1, -1, -1};
  // motion_constraint.dy = {0, 1, 0, -1, 1, -1, 1, -1};
  Dijikstra dijikstra(motion_constraint);
  std::vector<Coordinate> occupied_map = {{2, 2}, {2, 3}, {2, 4},
                                          {3, 2}, {3, 3}, {3, 4}};
  if (!dijikstra.SetOccupancyGrid(occupied_map, 6, 6)) {
    std::cout << "Invalid occupancy grid!" << std::endl;
  }
  if (!dijikstra.FindPath({0, 3}, {5, 1}))
    std::cout << "Could not find valid path!" << std::endl;

  return 0;
}