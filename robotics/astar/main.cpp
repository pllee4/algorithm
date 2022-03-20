/*
 * main.cpp
 *
 * Created on: Mar 19, 2022 20:56
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#include <iostream>

#include "astar/astar.hpp"

using namespace pllee4;
using namespace graph;

int main(int argc, char **argv) {
  AStar astar(AStar::MotionConstraintType::CARDINAL_MOTION);
  std::vector<Coordinate> occupied_map = {{2, 2}, {2, 3}, {2, 4},
                                          {3, 2}, {3, 3}, {3, 4}};
  if (!astar.SetOccupancyGrid(occupied_map, 6, 6)) {
    std::cout << "Invalid occupancy grid!" << std::endl;
  }
  if (!astar.FindPath({0, 3}, {5, 1}))
    std::cout << "Could not find valid path!" << std::endl;

  return 0;
}