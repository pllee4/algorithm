/*
 * main.cpp
 *
 * Created on: Mar 19, 2022 20:56
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#include <iostream>

#include "algorithm/robotics/astar/astar.hpp"

using namespace pllee4::graph;

int main(int argc, char **argv) {
  Astar astar(MotionConstraintType::CARDINAL_MOTION);
  astar.SetMapStorageSize(6, 6);
  std::vector<Coordinate> occupied_map = {{2, 2}, {2, 3}, {2, 4},
                                          {3, 2}, {3, 3}, {3, 4}};
  if (!astar.SetOccupiedGrid(occupied_map)) {
    std::cout << "Invalid occupancy grid!" << std::endl;
  }
  astar.SetStartAndDestination({0, 3}, {5, 1});
  if (!astar.FindPath()) std::cout << "Could not find valid path!" << std::endl;
  if (astar.GetPath().has_value()) {
    std::cout << "Found path" << std::endl;
    auto path = astar.GetPath().value();
    for (const auto coordinate : path) {
      std::cout << "traverse to (" << coordinate.x << "," << coordinate.y << ")"
                << std::endl;
    }
  }

  return 0;
}