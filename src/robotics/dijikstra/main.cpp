/*
 * main.cpp
 *
 * Created on: Mar 11, 2022 21:17
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#include <iostream>

#include "algorithm/robotics/dijikstra/dijikstra.hpp"

using namespace pllee4::graph;

int main(int argc, char **argv) {
  Dijikstra dijikstra(MotionConstraintType::CARDINAL_MOTION);
  dijikstra.SetMapStorageSize(6, 6);
  std::vector<Coordinate> occupied_map = {{2, 2}, {2, 3}, {2, 4},
                                          {3, 2}, {3, 3}, {3, 4}};
  if (!dijikstra.SetOccupiedGrid(occupied_map)) {
    std::cout << "Invalid occupancy grid!" << std::endl;
  }
  dijikstra.SetStartAndDestination({0, 3}, {5, 1});
  if (!dijikstra.FindPath())
    std::cout << "Could not find valid path!" << std::endl;
  if (dijikstra.GetPath().has_value()) {
    std::cout << "Found path" << std::endl;
    auto path = dijikstra.GetPath().value();
    for (const auto coordinate : path) {
      std::cout << "traverse to (" << coordinate.x << "," << coordinate.y << ")"
                << std::endl;
    }
  }

  return 0;
}